#!/usr/bin/env python3
"""组合"多串果串植株"追加部分：由 compose_plant.py 末尾调用 run()。

用独立文件避免 f-string 嵌套转义问题。
"""
import json
import math
import time

import numpy as np

from compose_plant import Geom, parse_dae, TOMATO_DAE, TRUSS_MESH_DIR, write_dae

OUT = None  # 由 run() 注入前先设默认
import pathlib
BASE = str(pathlib.Path(__file__).resolve().parent)
OUT = pathlib.Path(BASE) / 'aoc_tomato_farm_gazebo/models/22mx14m/tomato_plant_truss'

MAT_MAP = {'Branch1': 'branch', 'Leaf1': 'leaf1',
           'Leaf2': 'leaf2', 'Blossom3': 'blossom1'}
FRUIT_NODES = [('Fruit1', '#Branch1-mesh'), ('Fruit2', '#Branch1_004-mesh'),
               ('Fruit3', '#Branch1_006-mesh')]


def run(rng_seed=2026):
    rng = np.random.default_rng(rng_seed)
    geoms, nodes = parse_dae(TOMATO_DAE)
    print('[S1] tomato.dae 几何:', sorted(geoms.keys()))

    # 果串几何（truss 局部系）
    tfg = list(parse_dae(TRUSS_MESH_DIR / 'fruits.dae')[0].values())[0]
    tsg = list(parse_dae(TRUSS_MESH_DIR / 'stems.dae')[0].values())[0]
    truss_fruit_v, truss_fruit_t = tfg['verts'], tfg['tris']
    truss_stem_v, truss_stem_t = tsg['verts'], tsg['tris']
    top_z = max(v[2] for v in truss_stem_v)
    top_pts = [v for v in truss_stem_v if abs(v[2] - top_z) < 0.02]
    attach0 = np.array([np.mean([p[0] for p in top_pts]),
                        np.mean([p[1] for p in top_pts]), top_z])
    print('[S2] 果串: %d 果顶点, 顶部连接点 z=%.3f' % (len(truss_fruit_v), top_z))

    composed = {}
    markers = []

    # 保留茎/叶/花
    for node_name, mat in MAT_MAP.items():
        g = geoms[nodes[node_name]['geom']]
        G = Geom(node_name)
        G.verts = [tuple(v) for v in g['verts']]
        G.tris = [tuple(t) for t in g['tris']]
        composed[node_name] = G

    fruit_total = 0
    for ni, (fname, fgeom) in enumerate(FRUIT_NODES):
        F = geoms[fgeom]['verts']
        attach = F.mean(axis=0)
        yaw = rng.uniform(0, 2 * math.pi)
        c, sy = math.cos(yaw), math.sin(yaw)

        def xform(v):
            dx, dy, dz = v[0] - attach0[0], v[1] - attach0[1], v[2] - attach0[2]
            return (c * dx - sy * dy + attach[0],
                    sy * dx + c * dy + attach[1],
                    dz + attach[2])

        tf_v = [xform(v) for v in truss_fruit_v]
        ts_v = [xform(v) for v in truss_stem_v]

        Gf = Geom(fname + '_truss_fruits')
        Gf.verts = tf_v
        Gf.tris = [tuple(t) for t in truss_fruit_t]
        Gs = Geom(fname + '_truss_stems')
        Gs.verts = ts_v
        Gs.tris = [tuple(t) for t in truss_stem_t]
        composed[fname + '_truss_fruits'] = Gf
        composed[fname + '_truss_stems'] = Gs

        n_per = max(1, len(tf_v) // 12)
        for fi in range(12):
            seg = tf_v[fi * n_per * 2: (fi + 1) * n_per * 2]
            if len(seg) == 0:
                continue
            cc = np.array(seg).mean(axis=0)
            markers.append({'marker_type': 'FRUIT', 'truss_id': ni,
                            'translation': [float(cc[0]), float(cc[1]), float(cc[2])]})
        markers.append({'marker_type': 'PEDICEL', 'truss_id': ni,
                        'translation': [float(attach[0]), float(attach[1]), float(attach[2] + 0.02)]})
        fruit_total += 12
        print('[S3] %s: 果串挂于 (%.3f,%.3f,%.3f) yaw=%.2f' % (fname, *attach, yaw))

    print('[S4] 合并几何 %d 件, 果实 %d 颗' % (len(composed), fruit_total))
    (OUT / 'meshes').mkdir(parents=True, exist_ok=True)
    write_dae(OUT / 'meshes' / 'plant_truss.dae', composed)

    visuals = []
    for gname, G in composed.items():
        if '_truss_fruits' in gname:
            mat = ('<ambient>0.82 0.10 0.06 1</ambient>'
                   '<diffuse>0.82 0.10 0.06 1</diffuse>'
                   '<specular>0.4 0.2 0.1 1</specular>')
        elif '_truss_stems' in gname:
            mat = ('<ambient>0.22 0.40 0.14 1</ambient>'
                   '<diffuse>0.22 0.40 0.14 1</diffuse>')
        else:
            mname = MAT_MAP[gname]
            mat = ('<script><uri>model://tomato_plant_truss/materials/scripts/</uri>'
                   '<uri>model://tomato_plant_truss/materials/textures/</uri>'
                   '<name>' + mname + '</name></script>')
        visuals.append(
            '      <visual name="' + gname + '">\n'
            '        <geometry><mesh>'
            '<uri>model://tomato_plant_truss/meshes/plant_truss.dae</uri>'
            '<submesh><name>' + gname + '</name></submesh>'
            '</mesh></geometry>\n'
            '        <material>' + mat + '</material>\n'
            '      </visual>')
    sdf = ('<?xml version="1.0"?>\n<sdf version="1.6">\n'
           '  <model name="tomato_plant_truss">\n    <static>true</static>\n'
           '    <link name="link">\n'
           '      <collision name="trunk_col">\n'
           '        <geometry><cylinder><radius>0.03</radius>'
           '<length>1.3</length></cylinder></geometry>\n'
           '        <pose>0 0 0.65 0 0 0</pose>\n      </collision>\n'
           + '\n'.join(visuals) + '\n    </link>\n  </model>\n</sdf>\n')
    (OUT / 'model.sdf').write_text(sdf)
    (OUT / 'model.config').write_text(
        '<?xml version="1.0"?>\n<model>\n  <name>tomato_plant_truss</name>\n'
        '  <version>1.0</version>\n  <sdf version="1.6">model.sdf</sdf>\n'
        '  <author><name>compose_plant.py</name><email>local</email></author>\n'
        '  <description>多串果串植株：tomato_0 茎叶花 + tomato1_truss 果串</description>\n</model>\n')
    (OUT / 'markers.json').write_text(json.dumps(markers, indent=2))
    print('[S5-S6] model.sdf / markers.json (%d markers) 写出完成' % len(markers))
    return True

if __name__ == "__main__":
    run()
