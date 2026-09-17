"""Render normalized baseline/final mesh views for a visual comparison sheet."""
import os;os.environ.setdefault('PYOPENGL_PLATFORM','egl')
from pathlib import Path
import cv2,numpy as np,pyrender,trimesh

ROOT=Path(__file__).resolve().parents[1]/'results/real2sim_soup_scan_compare'

def look_at(eye,target):
    backward=(eye-target);backward/=np.linalg.norm(backward)
    right=np.cross([0,0,1.],backward);right/=np.linalg.norm(right)
    up=np.cross(backward,right)
    T=np.eye(4);T[:3,:3]=np.column_stack([right,up,backward]);T[:3,3]=eye;return T

def render(path, textured=False):
    src=trimesh.load(path,force='mesh',process=False);c=src.bounding_box.centroid
    # Use the reconstructed geometry with a neutral material here.  A clay render
    # makes holes and high-frequency surface noise visible without exposure or UV
    # atlas differences hiding the surface against the background.
    if textured:
        m=src.copy();m.apply_translation(-c)
    else:
        m=trimesh.Trimesh(vertices=src.vertices-c,faces=src.faces,process=False)
    scale=float(np.max(m.extents));scene=pyrender.Scene(bg_color=[35,39,46,255],ambient_light=[.24,.24,.24])
    material=pyrender.MetallicRoughnessMaterial(baseColorFactor=[0.72,0.76,0.82,1.0],metallicFactor=0.05,roughnessFactor=0.72,doubleSided=True)
    scene.add(pyrender.Mesh.from_trimesh(m,smooth=False,material=None if textured else material));cam=pyrender.PerspectiveCamera(yfov=np.deg2rad(38));light=pyrender.DirectionalLight(color=np.ones(3),intensity=3.5)
    views=[]
    for az in (-45,35,125):
        a=np.deg2rad(az);eye=np.array([np.cos(a),np.sin(a),.35])*scale*2.8;pose=look_at(eye,np.zeros(3));cn=scene.add(cam,pose=pose);ln=scene.add(light,pose=pose)
        renderer=pyrender.OffscreenRenderer(480,480)
        flags=pyrender.RenderFlags.FLAT if textured else pyrender.RenderFlags.NONE
        color,_=renderer.render(scene,flags=flags);renderer.delete();views.append(color);scene.remove_node(cn);scene.remove_node(ln)
    return np.hstack(views)

base=render(ROOT/'baseline/textured/textured_mesh.obj');dual=render(ROOT/'dual/tsdf_textured/textured_mesh.obj')
sheet=np.vstack([base,dual]);cv2.putText(sheet,'12-view single pass baseline',(18,38),cv2.FONT_HERSHEY_SIMPLEX,1,(245,245,245),2);cv2.putText(sheet,'24-view orthogonal pass + BundleSDF tracking + TSDF',(18,518),cv2.FONT_HERSHEY_SIMPLEX,.9,(245,245,245),2)
out=ROOT/'scan_compare.png';cv2.imwrite(str(out),sheet[...,::-1]);print(out)
base_tex=render(ROOT/'baseline/textured/textured_mesh.obj',textured=True);dual_tex=render(ROOT/'dual/tsdf_textured/textured_mesh.obj',textured=True)
texture_sheet=np.vstack([base_tex,dual_tex]);cv2.putText(texture_sheet,'12-view texture baseline',(18,38),cv2.FONT_HERSHEY_SIMPLEX,1,(245,245,245),2);cv2.putText(texture_sheet,'24-view occlusion-aware RGB texture bake',(18,518),cv2.FONT_HERSHEY_SIMPLEX,.9,(245,245,245),2)
tex_out=ROOT/'texture_compare.png';cv2.imwrite(str(tex_out),texture_sheet[...,::-1]);print(tex_out)
