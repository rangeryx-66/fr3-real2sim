"""Occlusion-aware texture rebaking from accepted RGB-D frames.

Geometry and poses come from BundleSDF.  eval_gt is never opened.
"""
from __future__ import annotations
import argparse,json
from pathlib import Path
import cv2,numpy as np,trimesh,xatlas
from scipy.spatial import cKDTree


def _bilinear(image,uv):
    h,w=image.shape[:2];x=np.clip(uv[:,0],0,w-1.001);y=np.clip(uv[:,1],0,h-1.001)
    x0=np.floor(x).astype(int);y0=np.floor(y).astype(int);x1=np.minimum(x0+1,w-1);y1=np.minimum(y0+1,h-1)
    wx=(x-x0)[:,None];wy=(y-y0)[:,None]
    return (1-wy)*((1-wx)*image[y0,x0]+wx*image[y0,x1])+wy*((1-wx)*image[y1,x0]+wx*image[y1,x1])


def collect_vertex_colors(mesh, dataset:Path, tracking:Path):
    V=np.asarray(mesh.vertices);N=np.asarray(mesh.vertex_normals);acc=np.zeros((len(V),3));weights=np.zeros(len(V));seen=np.zeros(len(V),int);pass_seen={}
    poses=sorted((dataset/'poses').glob('*.json'));lumas=[]
    for f in poses:
        row=json.loads(f.read_text());rgb=cv2.imread(str(dataset/'rgb'/f'{f.stem}.png'))
        mask=cv2.imread(str(dataset/'masks'/f'{f.stem}.png'),0)>0
        if mask.any():lumas.append(float(cv2.cvtColor(rgb,cv2.COLOR_BGR2LAB)[...,0][mask].mean()))
    # The scan station can be globally under-exposed.  Matching every image to
    # the under-exposed batch median preserves that failure in the atlas, so use
    # a conservative display mid-tone floor while retaining a bounded gain.
    target_luma=max(float(np.percentile(lumas,75)),96.) if lumas else 128.
    per_frame=[]
    for f,luma in zip(poses,lumas):
        row=json.loads(f.read_text());stem=f.stem;pose_file=tracking/f'{stem}.txt'
        if not pose_file.exists():continue
        T=np.loadtxt(pose_file);K=np.loadtxt(dataset/'cam_K.txt');R=T[:3,:3];t=T[:3,3]
        C=V@R.T+t;z=C[:,2];uv=np.c_[K[0,0]*C[:,0]/z+K[0,2],K[1,1]*C[:,1]/z+K[1,2]]
        rgb=cv2.imread(str(dataset/'rgb'/f'{stem}.png')).astype(float)[...,[2,1,0]]
        depth=cv2.imread(str(dataset/'depth'/f'{stem}.png'),cv2.IMREAD_UNCHANGED).astype(float)/1000
        mask=cv2.imread(str(dataset/'masks'/f'{stem}.png'),0)>0;hand=cv2.imread(str(dataset/'masks_hand'/f'{stem}.png'),0)>0
        h,w=mask.shape;ui=np.rint(uv[:,0]).astype(int);vi=np.rint(uv[:,1]).astype(int)
        inside=(z>.05)&(ui>=1)&(ui<w-1)&(vi>=1)&(vi<h-1);idx=np.flatnonzero(inside)
        if not len(idx):continue
        ncam=N[idx]@R.T;view=-C[idx]/np.linalg.norm(C[idx],axis=1,keepdims=True);cos=np.einsum('ij,ij->i',ncam,view)
        pix_ok=mask[vi[idx],ui[idx]]&~hand[vi[idx],ui[idx]]
        d=depth[vi[idx],ui[idx]];visible=pix_ok&(d>0)&(np.abs(d-z[idx])<.004)&(cos>.2)
        idx=idx[visible];cos=cos[visible]
        if not len(idx):continue
        exposure=np.clip(target_luma/max(luma,1.),.65,2.2);colors=np.clip(_bilinear(rgb,uv[idx])*exposure,0,255)
        wt=(cos**2)/np.maximum(z[idx]**2,.04)
        acc[idx]+=colors*wt[:,None];weights[idx]+=wt;seen[idx]+=1
        pid=str(row.get('pass_id',0));pass_seen.setdefault(pid,np.zeros(len(V),bool))[idx]=True
        per_frame.append({'frame':stem,'visible_vertices':int(len(idx)),'exposure_gain':float(exposure)})
    observed=weights>0;color=np.zeros((len(V),3),float);color[observed]=acc[observed]/weights[observed,None]
    # Fill unseen vertices only for texture continuity.  Geometry is unchanged and
    # the measured coverage fraction remains separately reported.
    if observed.any() and not observed.all():color[~observed]=color[observed][cKDTree(V[observed]).query(V[~observed])[1]]
    return np.clip(color,0,255).astype(np.uint8),observed,seen,per_frame,pass_seen,target_luma


def _raster_triangle(tex,filled,tri,cols):
    lo=np.maximum(np.floor(tri.min(0)).astype(int),0);hi=np.minimum(np.ceil(tri.max(0)).astype(int),np.array([tex.shape[1]-1,tex.shape[0]-1]))
    if np.any(hi<lo):return
    x=np.arange(lo[0],hi[0]+1);y=np.arange(lo[1],hi[1]+1);xx,yy=np.meshgrid(x,y);p=np.c_[xx.ravel()+.5,yy.ravel()+.5]
    a,b,c=tri;den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
    if abs(den)<1e-8:return
    w0=((b[1]-c[1])*(p[:,0]-c[0])+(c[0]-b[0])*(p[:,1]-c[1]))/den
    w1=((c[1]-a[1])*(p[:,0]-c[0])+(a[0]-c[0])*(p[:,1]-c[1]))/den;w2=1-w0-w1
    keep=(w0>=-.01)&(w1>=-.01)&(w2>=-.01);p=p[keep].astype(int);wgt=np.c_[w0[keep],w1[keep],w2[keep]]
    tex[p[:,1],p[:,0]]=np.clip(wgt@cols,0,255).astype(np.uint8);filled[p[:,1],p[:,0]]=255


def bake(mesh_path:Path,dataset:Path,reconstruction:Path,output:Path,resolution=2048):
    mesh=trimesh.load(mesh_path,force='mesh',process=False);colors,observed,seen,per_frame,pass_seen,target_luma=collect_vertex_colors(mesh,dataset,reconstruction/'ob_in_cam')
    vmapping,faces,uv=xatlas.parametrize(np.asarray(mesh.vertices,np.float32),np.asarray(mesh.faces,np.uint32))
    atlas_vertices=np.asarray(mesh.vertices)[vmapping]
    uv_px=np.c_[uv[:,0]*(resolution-1),(1-uv[:,1])*(resolution-1)];atlas=np.zeros((resolution,resolution,3),np.uint8);filled=np.zeros((resolution,resolution),np.uint8)
    atlas_colors=colors[vmapping]
    for face in faces:_raster_triangle(atlas,filled,uv_px[face],atlas_colors[face].astype(float))
    # Dilate color a few pixels into atlas gutters without changing coverage stats.
    for _ in range(4):
        missing=(filled==0);dil=cv2.dilate(atlas,np.ones((3,3),np.uint8));grow=missing&(cv2.dilate(filled,np.ones((3,3),np.uint8))>0);atlas[grow]=dil[grow];filled[grow]=128
    output=Path(output);output.mkdir(parents=True,exist_ok=True);tex=output/'material_0.png';cv2.imwrite(str(tex),atlas[...,[2,1,0]])
    obj=output/'textured_mesh.obj';mtl=output/'material_0.mtl'
    with obj.open('w') as out:
        out.write('mtllib material_0.mtl\n')
        for v in atlas_vertices:out.write(f'v {v[0]:.9g} {v[1]:.9g} {v[2]:.9g}\n')
        for t in uv:out.write(f'vt {t[0]:.9g} {t[1]:.9g}\n')
        out.write('usemtl material_0\n')
        for f in faces+1:out.write(f'f {f[0]}/{f[0]} {f[1]}/{f[1]} {f[2]}/{f[2]}\n')
    mtl.write_text('newmtl material_0\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd material_0.png\n')
    empty_seen=np.zeros(len(mesh.vertices),bool);p0=pass_seen.get('0',empty_seen);p1=pass_seen.get('1',empty_seen)
    report={'schema':'fr3_texture_bake/v1','source_mesh':str(mesh_path),'frames_used':len(per_frame),
            'measured_vertex_coverage':float(observed.mean()),'vertices_seen_ge_2':float(np.mean(seen>=2)),
            'atlas_surface_texel_fraction':float(np.mean(filled>0)),'mean_observations_per_vertex':float(seen.mean()),
            'vertex_coverage_by_pass':{k:float(v.mean()) for k,v in pass_seen.items()},
            'second_pass_new_vertex_fraction':float(np.mean(p1 & ~p0)),
            'exposure_policy':{'color_space':'CIELAB L*','target_luma':target_luma,'gain_limits':[.65,2.2]},
            'per_frame':per_frame,
            'products':{'textured_mesh':str(obj),'texture':str(tex),'material':str(mtl)}}
    (output/'texture_report.json').write_text(json.dumps(report,indent=2));return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mesh',type=Path);p.add_argument('dataset',type=Path);p.add_argument('reconstruction',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    print(json.dumps(bake(a.mesh,a.dataset,a.reconstruction,a.output),indent=2))
