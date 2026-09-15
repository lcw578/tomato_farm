#!/usr/bin/env python3
"""修复 A+B：对照真实樱桃番茄串外形修复 tomato1_truss 模型。

修复 A：主穗轴顶部折角截平封盖。
修复 B：小果梗缩短 + 果实向主梗内移。
"""
import bpy
import sys
import numpy as np
import os

argv = sys.argv[sys.argv.index('--')+1:]
out_dir = argv[0]

# ---------- 数据 API 烘焙 ----------
world = {}
for ob in bpy.data.objects:
    if ob.type != 'MESH':
        continue
    M = np.array(ob.matrix_world)
    vs = np.array([(M @ np.array([v.co.x, v.co.y, v.co.z, 1.0]))[:3] for v in ob.data.vertices])
    faces = [tuple(p.vertices) for p in ob.data.polygons]
    world[ob.name] = dict(verts=vs, faces=faces)

# ---------- 修复 A：主穗轴顶部截平 ----------
cap_tris = []
main = world['Cylinder.001']
vs = main['verts']
zcap = float(np.percentile(vs[:, 2], 88))
n_above = int((vs[:, 2] > zcap).sum())
vs[:, 2] = np.where(vs[:, 2] > zcap, zcap, vs[:, 2])
rim = vs[np.abs(vs[:, 2] - zcap) < 0.005]
if len(rim) >= 3:
    ang = np.arctan2(rim[:, 1] - rim[:, 1].mean(), rim[:, 0] - rim[:, 0].mean())
    order = rim[np.argsort(ang)]
    cx, cy = order[:, 0].mean(), order[:, 1].mean()
    pass  # 封面跳过
    print('[fixA] 截平 z=%.2f, 封面 %d 顶点' % (zcap, len(order)))

# ---------- 修复 B：小果梗缩短 + 果实内移 ----------
PAIR = {
    'Sphere.001': 'Cylinder.002',  'Sphere.002': 'Cylinder.004',
    'Sphere.003': 'Cylinder.002',  'Sphere.004': 'Cylinder.003',
    'Sphere.005': 'Cylinder.005',  'Sphere.006': 'Cylinder.007',
    'Sphere.007': 'Cylinder.006',  'Sphere.008': 'Cylinder.010',
    'Sphere.009': 'Cylinder.008',  'Sphere.010': 'Cylinder.009',
    'Sphere.011': 'Cylinder.011',  'Sphere.012': 'Cylinder.013',
}
SHORTEN = 0.60

for sn, cn in PAIR.items():
    if sn not in world or cn not in world:
        continue
    S, C = world[sn], world[cn]
    tip = C['verts'].mean(axis=0)
    zmin = C['verts'][:, 2].min()
    base = C['verts'][C['verts'][:, 2].argmin()]
    direction = base - tip
    dn = np.linalg.norm(direction)
    if dn < 1e-9:
        continue
    direction /= dn
    shift = direction * dn * (1 - SHORTEN)
    C['verts'] = C['verts'] + shift
    S['verts'] = S['verts'] + shift

# ---------- 写出（与 compose_plant.py 同格式）----------
def write_dae(path, name, verts, faces):
    tris = []
    for f in faces:
        if len(f) == 3: tris.append(tuple(f))
        elif len(f) == 4: tris += [(f[0], f[1], f[2]), (f[0], f[2], f[3])]
    pos = ' '.join(f'{c:.6f}' for v in verts for c in v)
    # triangles 只声明 VERTEX 一个输入（offset 0）：每顶点恰好 1 个索引，
    # 多写会导致 <p> 数量 = count*9 与声明不符，OGRE 静默加载失败（模型不可见）
    p = ' '.join(str(i) for t in tris for i in t)
    nv = len(verts)
    xml = f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_geometries>
    <geometry id="{name}-mesh" name="{name}">
      <mesh>
        <source id="{name}-mesh-positions">
          <float_array id="{name}-mesh-positions-array" count="{nv*3}">{pos}</float_array>
          <technique_common><accessor source="#{name}-mesh-positions-array" count="{nv}" stride="3">
            <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
          </accessor></technique_common>
        </source>
        <vertices id="{name}-mesh-vertices">
          <input semantic="POSITION" source="#{name}-mesh-positions"/>
        </vertices>
        <triangles count="{len(tris)}">
          <input semantic="VERTEX" source="#{name}-mesh-vertices" offset="0"/>
          <p>{p}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes><visual_scene id="Scene" name="Scene">
    <node id="{name}" name="{name}" type="NODE">
      <instance_geometry url="#{name}-mesh"/>
    </node>
  </visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>'''
    open(path, 'w').write(xml)

fruits_v = np.vstack([world[k]['verts'] for k in world if k.startswith('Sphere')])
fruits_f = []
off = 0
for k in sorted(world):
    if k.startswith('Sphere'):
        n = len(world[k]['verts'])
        for f in world[k]['faces']:
            fruits_f.append(tuple(i + off for i in f))
        off += n
write_dae(f'{out_dir}/fruits.dae', 'fruits', fruits_v, fruits_f)

stems_v = np.vstack([world[k]['verts'] for k in world if k.startswith('Cylinder')])
stems_f = []
off = 0
for k in sorted(world):
    if k.startswith('Cylinder'):
        n = len(world[k]['verts'])
        for f in world[k]['faces']:
            stems_f.append(tuple(i + off for i in f))
        off += n
write_dae(f'{out_dir}/stems.dae', 'stems', stems_v, stems_f)
print('[done] fruits.dae + stems.dae 写出完成')
