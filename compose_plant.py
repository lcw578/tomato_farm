#!/usr/bin/env python3
"""组合"多串果串植株"：tomato_0 的茎叶花（仓库原资产）+ tomato1_truss 果串（Unity 资产转换）。

原理：解析 tomato.dae 的几何与节点树，把 3 个 Fruit 节点（原始散果挂点）替换为
3 个 tomato1_truss 果串实例（变换到挂点、随机朝向），茎叶花几何原样保留，
合并写出一个 plant_truss.dae + model.sdf + markers.json。
"""
import re
import json
import math
import time
import numpy as np
from pathlib import Path

BASE = str(Path(__file__).resolve().parent)
TOMATO_DAE = Path(BASE) / 'aoc_tomato_farm_gazebo/models/22mx14m/tomato_0/meshes/tomato.dae'
TRUSS_MESH_DIR = Path(BASE) / 'aoc_tomato_farm_gazebo/models/22mx14m/tomato1_truss/meshes'
OUT_DIR = Path(BASE) / 'aoc_tomato_farm_gazebo/models/22mx14m/tomato_plant_truss'

# ---------------- DAE 解析 ----------------

def parse_dae(path):
    s = path.read_text()
    # geometry: id -> (positions, triangles p 数据, stride)
    geoms = {}
    for gm in re.finditer(r'<geometry id="([^"]+)".*?</geometry>', s, re.S):
        gid, body = gm.group(1), gm.group(0)
        pm = re.search(r'<source id="[^"]*positions"[^>]*>\s*<float_array[^>]*>([^<]+)</float_array>', body)
        if not pm:
            continue
        V = np.array([float(x) for x in pm.group(1).split()]).reshape(-1, 3)
        inputs = re.findall(r'<input semantic="(\w+)"[^>]*source="#([^"]+)"[^>]*offset="(\d+)"', body)
        voff = int([i for i in inputs if i[0] == 'VERTEX'][0][2])
        stride = max(int(i[2] or 0) for i in inputs) + 1
        tris = []
        for tm in re.finditer(r'<p>([^<]+)</p>', body):
            p = np.array([int(x) for x in tm.group(1).split()]).reshape(-1, stride)
            vt = p[:, voff]
            tris += [tuple(t) for t in vt.reshape(-1, 3).tolist()]
        geoms['#' + gid] = dict(verts=V, tris=tris, name=gid)
    # 节点: name -> matrix, geometry url, material target
    nodes = {}
    for nm in re.finditer(r'<node[^>]*name="([^"]+)"[^>]*>(.*?)</node>', s, re.S):
        name, body = nm.group(1), nm.group(2)
        mm = re.search(r'<matrix[^>]*>([^<]+)</matrix>', body)
        matrix = np.array([float(x) for x in mm.group(1).split()]).reshape(4, 4) if mm else np.eye(4)
        ig = re.search(r'<instance_geometry url="#([^"]+)"', body)
        mt = re.search(r'<instance_material symbol="[^"]*" target="([^"]*)"', body)
        if ig:
            nodes[name] = dict(matrix=matrix, geom='#' + ig.group(1),
                               material=mt.group(1) if mt else None)
    return geoms, nodes

# ---------------- 写出工具（与已验证渲染的生成器同一套格式）----------------

class Geom:
    def __init__(self, name):
        self.name = name
        self.verts, self.normals, self.uvs, self.tris = [], [], [], []

    def add_tri(self, a, b, c, na, nb, nc, ua, ub, uc):
        bv, bn, bt = len(self.verts), len(self.normals), len(self.uvs)
        self.verts += [a, b, c]
        self.normals += [na, nb, nc]
        self.uvs += [ua, ub, uc]
        self.tris.append((bv, bn, bt, bv+1, bn+1, bt+1, bv+2, bn+2, bt+2))

    def to_xml(self):
        pos = ' '.join(f'{c:.6f}' for v in self.verts for c in v)
        nrm = ' '.join(f'{c:.4f}' for v in self.normals for c in v)
        uv = ' '.join(f'{c:.4f}' for v in self.uvs for c in v)
        p = ' '.join(' '.join(str(x) for x in tri) for tri in self.tris)
        nv, nn, nt = len(self.verts), len(self.normals), len(self.uvs)
        return f'''    <geometry id="{self.name}-mesh" name="{self.name}">
      <mesh>
        <source id="{self.name}-mesh-positions">
          <float_array id="{self.name}-mesh-positions-array" count="{nv*3}">{pos}</float_array>
          <technique_common><accessor source="#{self.name}-mesh-positions-array" count="{nv}" stride="3">
            <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
          </accessor></technique_common>
        </source>
        <source id="{self.name}-mesh-normals">
          <float_array id="{self.name}-mesh-normals-array" count="{nn*3}">{nrm}</float_array>
          <technique_common><accessor source="#{self.name}-mesh-normals-array" count="{nn}" stride="3">
            <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
          </accessor></technique_common>
        </source>
        <source id="{self.name}-mesh-map-0">
          <float_array id="{self.name}-mesh-map-0-array" count="{nt*2}">{uv}</float_array>
          <technique_common><accessor source="#{self.name}-mesh-map-0-array" count="{nt}" stride="2">
            <param name="S" type="float"/><param name="T" type="float"/>
          </accessor></technique_common>
        </source>
        <vertices id="{self.name}-mesh-vertices">
          <input semantic="POSITION" source="#{self.name}-mesh-positions"/>
        </vertices>
        <triangles count="{len(self.tris)}">
          <input semantic="VERTEX" source="#{self.name}-mesh-vertices" offset="0"/>
          <input semantic="NORMAL" source="#{self.name}-mesh-normals" offset="1"/>
          <input semantic="TEXCOORD" source="#{self.name}-mesh-map-0" offset="2" set="0"/>
          <p>{p}</p>
        </triangles>
      </mesh>
    </geometry>'''

    def node_xml(self):
        return f'''      <node id="{self.name}" name="{self.name}" type="NODE">
        <matrix sid="transform">1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1</matrix>
        <instance_geometry url="#{self.name}-mesh"/>
      </node>'''


def write_dae(path, geoms):
    parts = [f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><contributor><authoring_tool>compose_plant</authoring_tool></contributor>
    <created>{time.strftime('%Y-%m-%dT%H:%M:%S')}</created>
    <unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>''']
    parts.append('  <library_geometries>')
    for g in geoms.values():
        parts.append(g.to_xml())
    parts.append('  </library_geometries>')
    parts.append('  <library_visual_scenes><visual_scene id="Scene" name="Scene">')
    for g in geoms.values():
        parts.append(g.node_xml())
    parts.append('  </visual_scene></library_visual_scenes>')
    parts.append('  <scene><instance_visual_scene url="#Scene"/></scene>')
    parts.append('</COLLADA>')
    path.write_text('\n'.join(parts))