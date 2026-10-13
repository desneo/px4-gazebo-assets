#!/usr/bin/env python3
"""生成非功能的静态艺术展示网格；不是飞行器工程或制造几何。"""
import math
from pathlib import Path
import xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1]
class Mesh:
    def __init__(self):self.vertices=[];self.faces=[]
    def point(self,p):self.vertices.append(p);return len(self.vertices)-1
    def grid(self,rows,wrap=False):
        ids=[[self.point(p) for p in row] for row in rows]
        for row,nextrow in zip(ids,ids[1:]):
            count=len(row) if wrap else len(row)-1
            for i in range(count):
                j=(i+1)%len(row);self.faces.extend([(row[i],nextrow[i],nextrow[j]),(row[i],nextrow[j],row[j])])
    def write(self,name):
        normals=[[0.,0.,0.] for p in self.vertices]
        for face in self.faces:
            a,b,c=[self.vertices[i] for i in face];u=[b[i]-a[i] for i in range(3)];v=[c[i]-a[i] for i in range(3)];n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
            for index in face:
                for i in range(3):normals[index][i]+=n[i]
        normals=[[v/max(1e-12,math.sqrt(sum(x*x for x in n))) for v in n] for n in normals]
        ns='http://www.collada.org/2005/11/COLLADASchema';E.register_namespace('',ns)
        def el(parent,tag,text=None,**attrs):
            n=E.SubElement(parent,'{'+ns+'}'+tag,attrs)
            if text is not None:n.text=text
            return n
        root=E.Element('{'+ns+'}COLLADA',version='1.4.1');asset=el(root,'asset');el(asset,'created','2026-10-13T00:00:00Z');el(asset,'modified','2026-10-13T00:00:00Z');el(asset,'unit',name='meter',meter='1');el(asset,'up_axis','Z_UP')
        geom=el(el(root,'library_geometries'),'geometry',id=name,name=name);mesh=el(geom,'mesh')
        for suffix,data in [('positions',self.vertices),('normals',normals)]:
            source=el(mesh,'source',id=name+'-'+suffix);el(source,'float_array',' '.join(f'{v:.7f}' for row in data for v in row),id=name+'-'+suffix+'-array',count=str(len(data)*3));accessor=el(el(source,'technique_common'),'accessor',source='#'+name+'-'+suffix+'-array',count=str(len(data)),stride='3')
            for axis in 'XYZ':el(accessor,'param',name=axis,type='float')
        verts=el(mesh,'vertices',id=name+'-vertices');el(verts,'input',semantic='POSITION',source='#'+name+'-positions')
        triangles=el(mesh,'triangles',count=str(len(self.faces)));el(triangles,'input',semantic='VERTEX',source='#'+name+'-vertices',offset='0');el(triangles,'input',semantic='NORMAL',source='#'+name+'-normals',offset='1');el(triangles,'p',' '.join(str(i) for face in self.faces for index in face for i in (index,index)))
        scenes=el(root,'library_visual_scenes');scene=el(scenes,'visual_scene',id='Scene');node=el(scene,'node',id=name+'-node');el(node,'instance_geometry',url='#'+name);el(el(root,'scene'),'instance_visual_scene',url='#Scene')
        path=ROOT/'meshes'/f'{name}.dae';E.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)

CONTROL=[(-1.45,.075,.08),(-1.2,.12,.12),(-.8,.18,.16),(-.2,.22,.19),(.4,.20,.18),(.8,.18,.16),(1.1,.15,.14),(1.4,.09,.10),(1.6,.025,.045),(1.65,.0005,.001)]
def radius(x):
    for i in range(len(CONTROL)-1):
        a,b=CONTROL[i:i+2]
        if x<=b[0]:
            t=max(0,(x-a[0])/(b[0]-a[0]));t=t*t*(3-2*t)
            return [a[j]*(1-t)+b[j]*t for j in (1,2)]
    return CONTROL[-1][1:]
def body_point(x,ang,offset=0):
    ry,rz=radius(x);return (x,(ry+offset)*math.cos(ang),.025+(rz+offset)*math.sin(ang))
def wing_point(y,v,sign=1):
    u=abs(y)/1.25;lead=.95-1.93*u**.86;trail=-1.37-.13*u; x=lead*(1-v)+trail*v;ry,rz=radius(x)
    pillow=(.016+.062*(1-u)**2)*math.sin(math.pi*v)**.68
    blend=rz*math.exp(-(y/max(.05,ry))**2)
    height=math.sqrt(pillow*pillow+blend*blend)
    return (x,y,.025+sign*height)
def tube(mesh,points,r):
    # 细小管线只作面板线装饰。
    rows=[]
    for p in points:
        rows.append([(p[0]+r*math.cos(a*math.pi/4),p[1]+r*math.sin(a*math.pi/4),p[2]) for a in range(8)])
    mesh.grid(rows,True)
def main():
    body=Mesh();body.grid([[body_point(-1.45+3.1*i/180,2*math.pi*j/96) for j in range(96)] for i in range(181)],True);body.faces=[tuple(reversed(f)) for f in body.faces];body.write('rounded_body')
    wing=Mesh()
    for sign in (1,-1):
        start=len(wing.faces);wing.grid([[wing_point(-1.25+2.5*i/140,j/100,sign) for j in range(101)] for i in range(141)])
        if sign<0:wing.faces[start:]=[tuple(reversed(f)) for f in wing.faces[start:]]
    wing.write('blended_delta')
    fins=Mesh()
    for side in (-1,1):
        for thickness_sign in (-1,1):
            start=len(fins.faces)
            fins.grid([[(-.96-.50*(z/.45),side*1.23+thickness_sign*.016*math.sin(math.pi*v)**.7,.025+z) if v==0 else ((-.96-.50*(z/.45))*(1-v)+(-1.5+.035*(z/.45))*v,side*1.23+thickness_sign*.016*math.sin(math.pi*v)**.7,.025+z) for v in [j/40 for j in range(41)]] for z in [.45*i/65 for i in range(66)]])
            if thickness_sign>0:fins.faces[start:]=[tuple(reversed(f)) for f in fins.faces[start:]]
    fins.write('wingtip_fins')
    panel=Mesh()
    for side in (-1,1):
        tube(panel,[tuple(c+(0 if j<2 else .002) for j,c in enumerate(wing_point(side*(.25+.94*i/100),.77))) for i in range(101)],.0014)
        tube(panel,[tuple(c+(0 if j<2 else .002) for j,c in enumerate(wing_point(side*.88,.08+.82*i/80))) for i in range(81)],.0012)
    tube(panel,[body_point(.84,2*math.pi*i/160,.0015) for i in range(161)],.0013);panel.write('decorative_panel_lines')
    # 顶部黑色小窗为纯视觉曲面补片。
    window=Mesh();rim=Mesh()
    for mesh,offset in ((rim,.011),(window,.0125)):
        mesh.grid([[(x,y,.025+radius(x)[1]*math.sqrt(max(0,1-(y/radius(x)[0])**2))+offset) for y in [(-.073 if mesh is rim else -.063)+( .146 if mesh is rim else .126)*j/32 for j in range(33)]] for x in [(1.00 if mesh is rim else 1.012)+(.285 if mesh is rim else .261)*i/40 for i in range(41)]])
    rim.write('window_frame');window.write('black_window')
    prop=Mesh()
    for side in (-1,1):
        rows=[]
        for i in range(41):
            r=.035+.34*i/40;width=.042*math.sin(math.pi*(.08+.84*i/40))
            rows.append([(-1.60+width*math.cos(a*math.pi/8)*.22,side*r,.025+width*math.sin(a*math.pi/8)) for a in range(16)])
        start=len(prop.faces);prop.grid(rows,True)
        if side<0:prop.faces[start:]=[tuple(reversed(f)) for f in prop.faces[start:]]
    prop.write('decorative_propeller')
    cap=Mesh();cap.grid([[(-1.40-.25*i/30,.065*math.sin(math.pi*i/30)*math.cos(j*math.pi/24),.025+.065*math.sin(math.pi*i/30)*math.sin(j*math.pi/24)) for j in range(48)] for i in range(31)],True);cap.write('tail_fairing')
    sdf=E.Element('sdf',version='1.9');model=E.SubElement(sdf,'model',name='delta_wing_display');E.SubElement(model,'static').text='true';link=E.SubElement(model,'link',name='artwork')
    colors={'rounded_body':'.86 .87 .88 1','blended_delta':'.87 .88 .89 1','wingtip_fins':'.86 .87 .88 1','decorative_panel_lines':'.28 .30 .33 1','window_frame':'.15 .17 .19 1','black_window':'.012 .022 .032 1','decorative_propeller':'.10 .12 .14 1','tail_fairing':'.26 .28 .30 1'}
    for name,color in colors.items():
        v=E.SubElement(link,'visual',name=name);E.SubElement(E.SubElement(E.SubElement(v,'geometry'),'mesh'),'uri').text=f'model://delta_wing_display/meshes/{name}.dae';mat=E.SubElement(v,'material');E.SubElement(mat,'ambient').text=color;E.SubElement(mat,'diffuse').text=color;E.SubElement(mat,'specular').text='.3 .3 .3 1';E.SubElement(mat,'shininess').text='65'
    E.indent(sdf,space='  ');E.ElementTree(sdf).write(ROOT/'model.sdf',encoding='utf-8',xml_declaration=True)
    (ROOT/'model.config').write_text('<?xml version="1.0"?><model><name>三角翼静态艺术展示</name><version>1.0</version><sdf version="1.9">model.sdf</sdf><author><name>px4-gazebo-assets</name></author><description>Non-functional static exterior artwork. No physics or flight systems.</description></model>\n')
if __name__=='__main__':main()
