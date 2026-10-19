"""Surface witnesses for colliding triangle pairs; avoids BVH vertex proxies."""
import numpy as np


def intersection_points(a,b,epsilon=1e-9):
    points=[]
    def inside(p,triangle):
        v0=triangle[1]-triangle[0];v1=triangle[2]-triangle[0];v2=p-triangle[0]
        d00=v0@v0;d01=v0@v1;d11=v1@v1;den=d00*d11-d01*d01
        if abs(den)<1e-25:return False
        u=(d11*(v2@v0)-d01*(v2@v1))/den;v=(d00*(v2@v1)-d01*(v2@v0))/den
        return u>=-1e-7 and v>=-1e-7 and u+v<=1+1e-7
    for first,second in [(a,b),(b,a)]:
        normal=np.cross(second[1]-second[0],second[2]-second[0]);length=np.linalg.norm(normal)
        if length<1e-15:continue
        normal/=length;distance=(first-second[0])@normal
        for i in range(3):
            j=(i+1)%3
            if abs(distance[i])<=epsilon and inside(first[i],second):points.append(first[i])
            if distance[i]*distance[j]<0:
                p=first[i]+distance[i]/(distance[i]-distance[j])*(first[j]-first[i])
                if inside(p,second):points.append(p)
    # Coplanar edge intersections can exist without contained vertices.
    normal=np.cross(a[1]-a[0],a[2]-a[0]);length=np.linalg.norm(normal)
    if length>1e-15 and np.max(np.abs((b-a[0])@(normal/length)))<=epsilon:
        drop=int(np.argmax(np.abs(normal)));aa=np.delete(a,drop,axis=1);bb=np.delete(b,drop,axis=1)
        for i in range(3):
            for j in range(3):
                x=aa[(i+1)%3]-aa[i];y=bb[(j+1)%3]-bb[j];M=np.column_stack((x,-y))
                if abs(np.linalg.det(M))<1e-20:continue
                t,u=np.linalg.solve(M,bb[j]-aa[i])
                if -1e-7<=t<=1+1e-7 and -1e-7<=u<=1+1e-7:points.append(a[i]+t*(a[(i+1)%3]-a[i]))
    return np.asarray(points).reshape(-1,3)
