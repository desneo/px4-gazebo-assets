#!/usr/bin/env python3
"""模型参数的外形示意图；不是 Gazebo 截图。"""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Ellipse,Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
R=Path(__file__).resolve().parents[1];p=json.loads((R/'parameters.json').read_text());sx=p['dimensions']['length_m']/.4;sy=p['dimensions']['span_m'];tri=[]
for path in (R/'meshes').glob('wing*.stl'):
 v=[list(map(float,l.split()[1:])) for l in path.read_text().splitlines() if l.startswith('vertex')];tri+=np.array(v).reshape(-1,3,3).tolist()
fig=plt.figure(figsize=(12,8),layout='constrained');ax=fig.add_subplot(221,projection='3d');ax.add_collection3d(Poly3DCollection(tri,facecolor='#eeeee8',edgecolor='#ccc',linewidth=.3))
for side in [-1,1]:
 y=side*.492*sy;ax.add_collection3d(Poly3DCollection([[(-.18*sx,y,0),(-.05*sx,y,0),(-.05*sx,y,.095),(-.18*sx,y,.025)]],facecolor='#138c19'))
 ax.plot([.18*sx,-.04*sx],[side*.025*sy,side*.5*sy],[.015,.015],color='#138719',linewidth=7)
 ax.plot([-.14*sx,-.14*sx],[side*.15*sy,side*.45*sy],[.008,.008],color='#33383d',linewidth=5)
u=np.linspace(0,2*np.pi,30);v=np.linspace(0,np.pi,20);ax.plot_surface(.025*sx+.175*sx*np.outer(np.cos(u),np.sin(v)),.075*sy*np.outer(np.sin(u),np.sin(v)),.012+.04*np.outer(np.ones_like(u),np.cos(v)),color='#159b23',linewidth=0)
ax.plot([.13*sx,.025*sx],[0,0],[.055,.06],color='#242b2d',linewidth=12);ax.plot([-.196*sx]*2,[-.07,.07],[.025,.025],color='#202020',linewidth=3)
ax.set(xlim=(-.22*sx,.22*sx),ylim=(-.52*sy,.52*sy),zlim=(-.06,.13),title='Perspective');ax.set_box_aspect((.44*sx,1.04*sy,.19));ax.view_init(35,35);ax.set_axis_off()
for index,title,xlab,ylab in [(222,'Top','y / span (m)','x / length (m)'),(223,'Front','y / span (m)','z / height (m)'),(224,'Side','x / length (m)','z / height (m)')]:
 a=fig.add_subplot(index);a.set(title=title,xlabel=xlab,ylabel=ylab);a.set_aspect('equal');a.grid(alpha=.2)
 if index==222:
  for side in [-1,1]:
   a.add_patch(Polygon([(side*.025*sy,.18*sx),(side*.5*sy,-.035*sx),(side*.5*sy,-.19*sx),(side*.025*sy,-.14*sx)],facecolor='#efefea',edgecolor='#999'))
   a.plot([side*.025*sy,side*.5*sy],[.18*sx,-.035*sx],color='#168b1c',lw=7);a.plot([side*.15*sy,side*.45*sy],[-.14*sx]*2,color='#333',lw=5)
  a.add_patch(Ellipse((0,.025*sx),.15*sy,.35*sx,facecolor='#159922'));a.add_patch(Ellipse((0,.07*sx),.07*sy,.13*sx,facecolor='#252a2d'));a.plot([-.07,.07],[-.196*sx]*2,color='#222',lw=3);a.set(xlim=(-.55*sy,.55*sy),ylim=(-.23*sx,.23*sx))
 elif index==223:
  a.add_patch(Rectangle((-.5*sy,-.011),sy,.022,fc='#eee',ec='#999'));a.add_patch(Ellipse((0,.012),.15*sy,.08,fc='#159922'));a.add_patch(Ellipse((0,.05),.07*sy,.022,fc='#252a2d'))
  for side in [-1,1]:a.add_patch(Rectangle((side*.492*sy-.006,0),.012,.095,fc='#138c19'))
  a.set(xlim=(-.55*sy,.55*sy),ylim=(-.05,.13))
 else:
  a.add_patch(Rectangle((-.19*sx,-.011),.37*sx,.022,fc='#eee',ec='#999'));a.add_patch(Ellipse((.025*sx,.012),.35*sx,.08,fc='#159922'));a.add_patch(Ellipse((.07*sx,.05),.13*sx,.022,fc='#252a2d'));a.add_patch(Polygon([(-.18*sx,0),(-.05*sx,0),(-.05*sx,.095),(-.18*sx,.025)],fc='#138c19'));a.plot([-.196*sx]*2,[-.045,.095],color='#222',lw=3);a.set(xlim=(-.23*sx,.23*sx),ylim=(-.06,.13))
fig.suptitle(f"Xiaobianfu | span {sy:.2f} m | overall length {.4*sx:.2f} m | total mass {p['mass_kg']:.2f} kg");fig.savefig(R/'docs/model_views.png',dpi=160)
