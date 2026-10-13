#!/usr/bin/env python3
"""从当前 SDF 绘制四个实际几何视图（不启动 Gazebo）。"""
import os
os.environ.setdefault('MPLCONFIGDIR','/tmp/xiaomotuo-matplotlib')
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1]
def transform(p):
    x,y,z,r,p,yaw=map(float,p.split());cr,sr=np.cos(r),np.sin(r);cp,sp=np.cos(p),np.sin(p);cy,sy=np.cos(yaw),np.sin(yaw)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]),np.array([x,y,z])
fig=plt.figure(figsize=(14,10),facecolor='#edf1f6');model=E.parse(ROOT/'model.sdf').getroot().find('model')
for idx,(elev,azim,title) in enumerate([(25,45,'Perspective'),(90,0,'Top'),(0,90,'Side'),(0,0,'Front')],1):
    ax=fig.add_subplot(2,2,idx,projection='3d');ax.set_title(title)
    for link in model.findall('link'):
        lp=link.findtext('pose','0 0 0 0 0 0');R,t=transform(lp)
        if link.get('name','').endswith('Wheel'):
            j=model.find("joint[child='"+link.get('name')+"']");R,t=transform(j.findtext('pose'))
        for v in link.findall('visual'):
            vr,vt=transform(v.findtext('pose','0 0 0 0 0 0'));g=v.find('geometry');faces=[]
            if g.find('box') is not None:
                size=np.array(list(map(float,g.findtext('box/size').split())))/2
                vertices=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*size
                faces=[vertices[list(i)] for i in [(0,1,3,2),(4,5,7,6),(0,4,6,2),(1,5,7,3),(0,1,5,4),(2,3,7,6)]]
            elif g.find('mesh') is not None:
                path=ROOT/'meshes'/Path(g.findtext('mesh/uri')).name
                if path.suffix=='.dae':
                    dae=E.parse(path);ns={'d':'http://www.collada.org/2005/11/COLLADASchema'}
                    source=next(s for s in dae.findall('.//d:source',ns) if s.get('id','').endswith('-positions'))
                    vertices=np.array(list(map(float,source.find('d:float_array',ns).text.split()))).reshape(-1,3)
                    scale=np.array(list(map(float,g.findtext('mesh/scale','1 1 1').split())));vertices*=scale
                    indices=np.array(list(map(int,dae.find('.//d:triangles/d:p',ns).text.split())))[::2].reshape(-1,3)
                    faces=vertices[indices]
                else:
                    verts=[list(map(float,line.split()[1:])) for line in path.read_text().splitlines() if line.startswith('vertex ')]
                    faces=np.array(verts).reshape(-1,3,3)
            elif g.find('cylinder') is not None:
                radius=float(g.findtext('cylinder/radius'));length=float(g.findtext('cylinder/length'));ang=np.linspace(0,2*np.pi,24)
                rings=[np.array([[radius*np.cos(a),radius*np.sin(a),z] for a in ang]) for z in (-length/2,length/2)]
                faces=[np.array([rings[0][i],rings[0][i+1],rings[1][i+1],rings[1][i]]) for i in range(23)]+rings
            if len(faces):
                color=list(map(float,v.findtext('material/diffuse','0.1 0.1 0.1 1').split()));polys=[(np.array(f)@vr.T+vt)@R.T+t for f in faces];ax.add_collection3d(Poly3DCollection(polys,facecolors=color,linewidths=0,shade=True))
    ax.set_xlim(-1.9,1.7);ax.set_ylim(-1.4,1.4);ax.set_zlim(-.6,.65);ax.set_box_aspect((3.6,2.8,1.25));ax.view_init(elev=elev,azim=azim);ax.set_xlabel('X forward (m)');ax.set_ylabel('Y left (m)');ax.set_zlabel('Z up (m)')
    if idx==2:ax.set_zticks([]);ax.set_zlabel('')
    elif idx==3:ax.set_yticks([]);ax.set_ylabel('')
    elif idx==4:ax.set_xticks([]);ax.set_xlabel('')
    ax.xaxis.labelpad=12;ax.yaxis.labelpad=12;ax.zaxis.labelpad=10
import json
cfg=json.loads((ROOT/'parameters.json').read_text())
fig.suptitle(f"Xiaomotuo | {cfg['length_m']:.2f} m length | {cfg['span_m']:.2f} m span | {cfg['mass_kg']:g} kg | decorative delta exterior",fontsize=15)
fig.tight_layout();fig.savefig(ROOT/'docs/model_views.png',dpi=160)
