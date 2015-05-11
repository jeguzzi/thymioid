#!/usr/bin/env python 

import subprocess


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
        

