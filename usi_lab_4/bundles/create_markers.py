#!/usr/bin/env python

import subprocess
import cairosvg

def svg_wings(a,l,s):
    svg_code=""
    for w in a:
        svg_code+="""
            <g transform='rotate({a} {rx} {ry})'>
                <path d='M 0 {s} l {l} {l} l {m} 0 l {l} -{l} Z' stroke='grey'/>
            </g>
""".format(l=l,s=s,m=s-2*l,a=-90*w,rx=s*0.5,ry=s*0.5)

    return svg_code

def svg_face(f,i,m,s):
    b=1
    faces=[
        {'x':b,'y':b+s,'a':[3]},
        {'x':b+s,'y':b+s,'a':[]},
        {'x':b+2*s,'y':b+s,'a':[]},
        {'x':b+3*s,'y':b+s,'a':[]},
        {'x':b,'y':b+2*s,'a':[0,1,3]},
        {'x':b,'y':b,'a':[1,2,3]}
    ]
    
    
    svg_code = """
        <g transform='translate({x},{y})'>
            <rect width="{s}" height="{s}"/>
            <image x="{d}" y="{d}" width="{m}" height="{m}" xlink:href="MarkerData_{i}.png"/>
            {a}
        </g>
    """.format(d=(s-m)*0.5,m=m,s=s,i=i,x=faces[f]['x'],y=faces[f]['y'],a=svg_wings(faces[f]['a'],b,s))
    
    return svg_code
    

def create_dice(i,m,s):

    svg_code = """<?xml version="1.0" standalone="no"?>
<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN"
  "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
<svg width="{w}cm" height="{h}cm" version="1.1" viewBox="0 0 {w} {h}" stroke="grey" stroke-width='0.05' fill="white"
     xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" >
""".format(w=s*5+1,h=s*3+2)
    for f in range(6):
        svg_code+=svg_face(f,6*i+f,m,s)
    svg_code+="""
</svg>
"""
    with open('dice_1.svg','w+') as f:
        print svg_code
        f.write(svg_code)
    with open('dice_1.png','w+') as f:
        cairosvg.svg2png(bytestring=svg_code,write_to=f)



def create_markers(n):
    for i in range(n):
        subprocess.call(['rosrun','ar_track_alvar','createMarker',str(i)])



def create_multimarker_xml(marker_size,dice_size,n):

    D=dice_size*0.5
    d=-D
    S=marker_size*0.5
    s=-S
    zD=-dice_size
    zd=0
    zS=-(dice_size+marker_size)*0.5
    zs=-(dice_size-marker_size)*0.5

    dice_vertices=[
        [[s,s,zd],[S,s,zd],[S,S,zd],[s,S,zd]],
        [[D,s,zs],[D,s,zS],[D,S,zS],[D,S,zs]],
        [[S,s,zD],[s,s,zD],[s,S,zD],[S,S,zD]],
        [[d,s,zS],[d,s,zs],[d,S,zs],[d,S,zS]],
        [[s,d,zS],[S,d,zS],[S,d,zs],[s,d,zs]],
        [[s,D,zs],[S,D,zs],[S,D,zS],[s,D,zS]]
    ]
    s="""<?xml version='1.0' encoding='UTF-8' standalone='no' ?>
    <multimarker markers='6'>"""
    for f in range(6):
        s+="""
        <marker index='{i}' status='1'>""".format(i=n*6+f)
        for c in range(4):
            v=dice_vertices[f][c]
            s+="""
            <corner x='{0}' y='{1}' z='{2}'/>""".format(*tuple(v))
        s+="""
        </marker>"""
    s+="""
    </multimarker>"""
    with open('multimarker_{0}.xml'.format(n), 'w') as f:
        f.write(s)

if __name__ == "__main__":
    n=20
    create_markers(6*n)
    for m in range(n):
        create_multimarker_xml(4,5,m)
        create_dice(6*m,4,5)