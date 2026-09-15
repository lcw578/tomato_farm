#!/usr/bin/env python3
"""番茄串（truss）植株纯 Python 生成器。

不依赖 Blender：直接产出 Gazebo Classic 可用的 DAE(model) + model.sdf + markers.json。
形态目标：单主干 + 若干结果节点，每个结果节点挂一条番茄串
（共同果梗弧线 + 5~9 个渐变果实 + 萼片），外观贴图复用项目内实物扫描纹理
（AG15frt*/AG15brn1/AG15blo*/AG15lef*，通过 model.sdf 的 ogre material script 绑定，
 与现有 tomato_N 模型同一套材质体系）。

用法：
  python3 truss_generator.py --out <model_dir> --seed 42 [--model-name tomato_truss_42]
      [--fruits-per-truss 6] [--trusses 6] [--preview preview.png]

markers.json 里每个果实一条记录（marker_type=FRUIT，带 truss_id），另加每串
果梗剪切点（marker_type=PEDICEL，剪切目标真值）。
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

# ---------------- DAE 写出工具 ----------------
DAE_HEADER = '''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor><authoring_tool>truss_generator.py</authoring_tool></contributor>
    <created>{created}</created>
    <modified>{created}</modified>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
'''

VEC_FMT = ' '.join(f'{v:.6f}' for v in [0])


class Geom:
    """一个 submesh（=DAE geometry+node，gazebo 以 node 名作 submesh 名）。"""

    def __init__(self, name):
        self.name = name
        self.verts = []      # [(x,y,z)]
        self.normals = []    # [(x,y,z)]
        self.uvs = []        # [(u,v)]
        self.tris = []       # [(vi,ni,ti) x3]

    def add_tri(self, a, b, c, na, nb, nc, ua, ub, uc):
        base_v, base_n, base_t = len(self.verts), len(self.normals), len(self.uvs)
        self.verts += [a, b, c]
        self.normals += [na, nb, nc]
        self.uvs += [ua, ub, uc]
        self.tris.append((base_v, base_n, base_t,
                          base_v + 1, base_n + 1, base_t + 1,
                          base_v + 2, base_n + 2, base_t + 2))

    def to_xml(self):
        pos = ' '.join(f'{c:.6f}' for v in self.verts for c in v)
        nrm = ' '.join(f'{c:.4f}' for v in self.normals for c in v)
        uv = ' '.join(f'{c:.4f}' for v in self.uvs for c in v)
        pidx = ' '.join(f'{tri[0]} {tri[1]} {tri[2]} {tri[3]} {tri[4]} {tri[5]} {tri[6]} {tri[7]} {tri[8]}'
                        for tri in self.tris)
        nv, nn, nt = len(self.verts), len(self.normals), len(self.uvs)
        return f'''
    <geometry id="{self.name}-mesh" name="{self.name}">
      <mesh>
        <source id="{self.name}-mesh-positions">
          <float_array id="{self.name}-mesh-positions-array" count="{nv*3}">{pos}</float_array>
          <technique_common>
            <accessor source="#{self.name}-mesh-positions-array" count="{nv}" stride="3">
              <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="{self.name}-mesh-normals">
          <float_array id="{self.name}-mesh-normals-array" count="{nn*3}">{nrm}</float_array>
          <technique_common>
            <accessor source="#{self.name}-mesh-normals-array" count="{nn}" stride="3">
              <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="{self.name}-mesh-map-0">
          <float_array id="{self.name}-mesh-map-0-array" count="{nt*2}">{uv}</float_array>
          <technique_common>
            <accessor source="#{self.name}-mesh-map-0-array" count="{nt}" stride="2">
              <param name="S" type="float"/><param name="T" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="{self.name}-mesh-vertices">
          <input semantic="POSITION" source="#{self.name}-mesh-positions"/>
        </vertices>
        <triangles count="{len(self.tris)}">
          <input semantic="VERTEX" source="#{self.name}-mesh-vertices" offset="0"/>
          <input semantic="NORMAL" source="#{self.name}-mesh-normals" offset="1"/>
          <input semantic="TEXCOORD" source="#{self.name}-mesh-map-0" offset="2" set="0"/>
          <p>{pidx}</p>
        </triangles>
      </mesh>
    </geometry>'''

    def node_xml(self):
        return f'''
      <node id="{self.name}" name="{self.name}" type="NODE">
        <matrix sid="transform">1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1</matrix>
        <instance_geometry url="#{self.name}-mesh"/>
      </node>'''


def rot_z(v, a):
    c, s = math.cos(a), math.sin(a)
    return [v[0] * c - v[1] * s, v[1] * c + v[0] * s, v[2]]


def rot_axis(v, axis, a):
    """Rodrigues 旋转。"""
    k = np.array(axis, dtype=float)
    k /= np.linalg.norm(k)
    v = np.asarray(v, dtype=float)
    return list(v * math.cos(a) + np.cross(k, v) * math.sin(a) + k * np.dot(k, v) * (1 - math.cos(a)))


def sphere_geom(name, center, radius, squash=0.92, dent=0.0, segments=14, rings=10,
                u_offset=0.0, color_band=None):
    """果实体：椭球(squash 压扁) + 顶部凹陷(dent)。UV 球面展开。"""
    g = Geom(name)
    cx, cy, cz = center
    for i in range(rings):
        th0 = math.pi * i / rings
        th1 = math.pi * (i + 1) / rings
        for j in range(segments):
            ph0 = 2 * math.pi * j / segments
            ph1 = 2 * math.pi * (j + 1) / segments

            def pt(th, ph):
                # 顶部（th 小）向内收形成果蒂凹陷
                r = radius * (1.0 - dent * max(0.0, 1.0 - th / 0.55))
                x = r * math.sin(th) * math.cos(ph)
                y = r * math.sin(th) * math.sin(ph)
                z = -r * math.cos(th) * squash   # 负号使顶部朝上(+z)为蒂端
                return [cx + x, cy + y, cz + z]

            def uv(th, ph):
                return (ph / (2 * math.pi) + u_offset, 1.0 - th / math.pi)

            a0, a1 = pt(th0, ph0), pt(th0, ph1)
            b0, b1 = pt(th1, ph0), pt(th1, ph1)
            ua0, ua1 = uv(th0, ph0), uv(th0, ph1)
            ub0, ub1 = uv(th1, ph0), uv(th1, ph1)

            def nrm(p):
                v = np.array(p) - np.array(center)
                v[2] /= squash
                n = np.linalg.norm(v)
                return list(v / n) if n > 1e-9 else [0, 0, 1]

            if i > 0:
                g.add_tri(a0, a1, b1, nrm(a0), nrm(a1), nrm(b1), ua0, ua1, ub1)
            if i < rings - 1:
                g.add_tri(a0, b1, b0, nrm(a0), nrm(b1), nrm(b0), ua0, ub1, ub0)
    return g


def tube_geom(name, points, radius0, radius1, slices=8):
    """沿折线的变径圆管（果梗/主干）。points: [(x,y,z)]。"""
    g = Geom(name)
    pts = [np.array(p, dtype=float) for p in points]
    rings = []
    for i, p in enumerate(pts):
        if i == 0:
            t = pts[1] - pts[0]
        elif i == len(pts) - 1:
            t = pts[-1] - pts[-2]
        else:
            t = pts[i + 1] - pts[i - 1]
        t = t / np.linalg.norm(t)
        ref = np.array([0, 0, 1.0]) if abs(t[2]) < 0.9 else np.array([1.0, 0, 0])
        u = np.cross(t, ref)
        u /= np.linalg.norm(u)
        w = np.cross(t, u)
        r = radius0 + (radius1 - radius0) * i / (len(pts) - 1)
        rings.append([p + (u * math.cos(2 * math.pi * j / slices) +
                           w * math.sin(2 * math.pi * j / slices)) * r
                      for j in range(slices)])

    def nrm_of(i, j):
        p = pts[i]
        return list((rings[i][j] - p) / np.linalg.norm(rings[i][j] - p))

    for i in range(len(pts) - 1):
        for j in range(slices):
            j2 = (j + 1) % slices
            a0, a1 = rings[i][j], rings[i][j2]
            b0, b1 = rings[i + 1][j], rings[i + 1][j2]
            u0, u1 = j / slices, (j + 1) / slices
            g.add_tri(a0, a1, b1, nrm_of(i, j), nrm_of(i, j2), nrm_of(i + 1, j2),
                      (u0, 0), (u1, 0), (u1, 1))
            g.add_tri(a0, b1, b0, nrm_of(i, j), nrm_of(i + 1, j2), nrm_of(i + 1, j),
                      (u0, 0), (u1, 1), (u0, 1))
    return g


def sepal_geom(name, center, radius, n=5, length=0.4):
    """果萼：n 片细长三角面片，从蒂端向外辐射（双面：正反两份三角）。"""
    g = Geom(name)
    cx, cy, cz = center
    z_top = cz + radius * 0.95
    for k in range(n):
        a = 2 * math.pi * k / n + 0.3
        tip = [cx + math.cos(a) * radius * length * 2.2,
               cy + math.sin(a) * radius * length * 2.2,
               z_top + radius * length * 0.9]
        left = [cx + math.cos(a - 0.45) * radius * 0.55,
                cy + math.sin(a - 0.45) * radius * 0.55,
                z_top - radius * 0.08]
        right = [cx + math.cos(a + 0.45) * radius * 0.55,
                 cy + math.sin(a + 0.45) * radius * 0.55,
                 z_top - radius * 0.08]
        nrm = [math.cos(a), math.sin(a), 0.35]
        uv = (k / n, 0.5)
        g.add_tri(left, tip, right, nrm, nrm, nrm, uv, uv, uv)
        g.add_tri(right, tip, left, [-c for c in nrm], [-c for c in nrm], [-c for c in nrm],
                  uv, uv, uv)
    return g


def leaf_geom(name, base, yaw, length=0.16, width=0.10, tilt=0.9):
    """叶面片：两三角形组成的菱形叶（贴 alpha 叶纹理，双面）。"""
    g = Geom(name)
    bx, by, bz = base
    tip = [bx + math.cos(yaw) * length * math.cos(tilt) * 0.4,
           by + math.sin(yaw) * length * math.cos(tilt) * 0.4,
           bz + length * math.sin(-tilt)]
    left = [bx + math.cos(yaw + math.pi / 2) * width,
            by + math.sin(yaw + math.pi / 2) * width,
            bz - 0.01]
    right = [bx + math.cos(yaw - math.pi / 2) * width,
             by + math.sin(yaw - math.pi / 2) * width,
             bz - 0.01]
    nrm = [math.cos(yaw + math.pi / 2), math.sin(yaw + math.pi / 2), 0.55]
    uv = (0.5, 0.5)
    g.add_tri(left, tip, right, nrm, nrm, nrm, uv, uv, uv)
    g.add_tri(right, tip, left, [-c for c in nrm], [-c for c in nrm], [-c for c in nrm],
              uv, uv, uv)
    return g


# ---------------- 植株生成 ----------------

def gen_plant(seed, fruits_per_truss=(5, 8), n_trusses=(6, 8),
              fruit_r=(0.024, 0.034), stem_h=(1.15, 1.35), drop_deg=(28, 55),
              name_prefix='p'):
    rng = np.random.default_rng(seed)
    truss_n = rng.integers(n_trusses[0], n_trusses[1] + 1)
    height = rng.uniform(*stem_h)

    geoms = {}
    markers = []

    # ---- 主干：轻微弯曲的锥形管 ----
    pts = []
    lean = rng.normal(0, 0.05, 3)
    for i in range(7):
        t = i / 6
        pts.append([lean[0] * t * t, lean[1] * t * t, height * t])
    geoms[f'{name_prefix}_trunk'] = tube_geom(f'{name_prefix}_trunk', pts, 0.009, 0.004)

    # ---- 节位与叶 ----
    node_zs = sorted(rng.uniform(0.22, height * 0.97, truss_n + rng.integers(2, 5)))
    leaf_budget = rng.integers(7, 11)
    for i, z in enumerate(sorted(rng.uniform(0.1, height * 0.98, leaf_budget))):
        yaw = rng.uniform(0, 2 * math.pi)
        geoms[f'{name_prefix}_leaf{i}'] = leaf_geom(f'{name_prefix}_leaf{i}', [0.008 * math.cos(yaw), 0.008 * math.sin(yaw), z],
                                      yaw, length=rng.uniform(0.13, 0.19),
                                      width=rng.uniform(0.07, 0.11),
                                      tilt=rng.uniform(0.7, 1.2))

    # ---- 果串 ----
    truss_zs = node_zs[:truss_n]
    fruit_total = 0
    for ti, z0 in enumerate(sorted(truss_zs, reverse=True)):
        yaw0 = rng.uniform(0, 2 * math.pi) + ti * 2.39996  # 黄金角散布
        n_fruit = rng.integers(fruits_per_truss[0], fruits_per_truss[1] + 1)
        drop = math.radians(rng.uniform(*drop_deg))
        r0 = rng.uniform(*fruit_r)
        length = r0 * 2 * (n_fruit * 0.75) * rng.uniform(0.9, 1.15)

        # 果梗曲线：从节点出发水平伸出后下垂
        d = [math.cos(yaw0), math.sin(yaw0)]
        arc = []
        for k in range(6):
            t = k / 5
            arc.append([d[0] * length * 0.45 * t,
                        d[1] * length * 0.45 * t,
                        z0 + length * math.sin(drop) * (t * t) * -1.0 + 0.02 * t])
        geoms[f'{name_prefix}_trussshot{ti}'] = tube_geom(f'{name_prefix}_trussshot{ti}', arc, 0.0045, 0.0028, slices=6)

        # 果实沿果梗交替排布（左右小幅交错），每果经小果梗(pedicel)连接到主梗
        for fi in range(n_fruit):
            t = 0.42 + 0.58 * fi / max(1, n_fruit - 1)
            # 主梗弧线上对应点（小果梗连接点）
            ax = d[0] * length * 0.45 * t
            ay = d[1] * length * 0.45 * t
            az = z0 + length * math.sin(drop) * (t * t) * -1.0 + 0.02 * t
            # 果实中心：连接点 + 小幅横向交错，沿下垂方向下坠使果顶贴住小果梗
            side = (1 if fi % 2 else -1)
            lat = 0.025 * side
            px = ax + d[1] * lat
            py = ay - d[0] * lat
            pr = r0 * (0.75 + 0.45 * t) * rng.uniform(0.92, 1.08)
            drop_v = pr * 0.72
            pz = az - drop_v
            name = f'{name_prefix}_fruit_t{ti}_f{fi}'
            geoms[name] = sphere_geom(name, [px, py, pz], pr, dent=0.22,
                                      u_offset=rng.uniform(0, 1))
            # 小果梗：主梗连接点 -> 果顶（果心上方 0.92*r）
            geoms[f'{name_prefix}_pedicel_t{ti}_f{fi}'] = tube_geom(
                f'{name_prefix}_pedicel_t{ti}_f{fi}',
                [[ax, ay, az], [px, py, pz + pr * 0.92]], 0.0018, 0.0012, slices=6)
            # 萼片贴果顶
            geoms[f'{name_prefix}_sepal_t{ti}_f{fi}'] = sepal_geom(
                f'{name_prefix}_sepal_t{ti}_f{fi}', [px, py, pz], pr)
            fruit_total += 1
            markers.append({
                'marker_type': 'FRUIT',
                'truss_id': ti,
                'translation': [float(px), float(py), float(pz)],
            })

        # 果梗剪切点（串基部的 pedicel 真值）
        markers.append({
            'marker_type': 'PEDICEL',
            'truss_id': ti,
            'translation': [float(d[0] * length * 0.18), float(d[1] * length * 0.18), float(z0 + 0.01)],
        })

    return geoms, markers, fruit_total, truss_n


# ---------------- SDF / 落盘 ----------------

SDF_TEMPLATE = '''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <collision name="collision_trunk">
        <geometry><cylinder><radius>0.012</radius><length>{stem_len:.3f}</length></cylinder></geometry>
        <pose>0 0 {stem_half:.3f} 0 0 0</pose>
      </collision>
{visuals}
    </link>
  </model>
</sdf>
'''

VISUAL_TEMPLATE = '''      <visual name="{gname}">
        <geometry>
          <mesh>
            <uri>model://{model_name}/meshes/{model_name}.dae</uri>
            <submesh><name>{gname}</name></submesh>
          </mesh>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
          <script>
            <uri>model://{model_name}/materials/scripts/</uri>
            <uri>model://{model_name}/materials/textures/</uri>
            <name>{mat_name}</name>
          </script>
        </material>
      </visual>'''


def material_for(gname):
    if '_fruit' in gname:
        return 'fruit'
    if '_sepal' in gname or '_blossom' in gname:
        return 'blossom'
    if '_trunk' in gname or '_truss' in gname or '_pedicel' in gname:
        return 'branch'
    return 'leaf1'


MODEL_CONFIG = '''<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>truss_generator</name><email>local</email></author>
  <description>Procedural tomato truss plant (pure-python)</description>
</model>
'''


def write_model(out_dir: Path, model_name, geoms, markers, stem_len):
    (out_dir / 'meshes').mkdir(parents=True, exist_ok=True)
    (out_dir / 'materials' / 'scripts').mkdir(parents=True, exist_ok=True)
    (out_dir / 'materials' / 'textures').mkdir(parents=True, exist_ok=True)

    # DAE
    parts = [DAE_HEADER.format(created=time.strftime('%Y-%m-%dT%H:%M:%S'))]
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
    (out_dir / 'meshes' / f'{model_name}.dae').write_text('\n'.join(parts))

    # SDF（按材质合并 visual 数量：fruit 类合并会丢失 submesh 区分，直接每几何一个 visual）
    visuals = '\n'.join(
        VISUAL_TEMPLATE.format(gname=g, model_name=model_name, mat_name=material_for(g))
        for g in geoms)
    (out_dir / 'model.sdf').write_text(SDF_TEMPLATE.format(
        model_name=model_name, visuals=visuals, stem_len=stem_len, stem_half=stem_len / 2))
    (out_dir / 'model.config').write_text(MODEL_CONFIG.format(name=model_name))

    # markers
    (out_dir / 'markers.json').write_text(json.dumps(markers, indent=2))
    return len(markers)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--model-name', default=None)
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--fruits-per-truss', default='5,8')
    ap.add_argument('--trusses', default='6,8')
    ap.add_argument('--fruit-r', default='0.024,0.034')
    args = ap.parse_args()

    out = Path(args.out)
    name = args.model_name or f'tomato_truss_{args.seed}'
    fpf = tuple(int(x) for x in args.fruits_per_truss.split(','))
    tr = tuple(int(x) for x in args.trusses.split(','))
    fr = tuple(float(x) for x in args.fruit_r.split(','))

    geoms, markers, fruit_total, truss_n = gen_plant(args.seed, fpf, tr, fr,
                                                     name_prefix=name)
    n = write_model(out, name, geoms, markers, stem_len=1.2)
    print(f'模型 {name}: {truss_n} 串 / {fruit_total} 果 / {len(geoms)} 几何件 / markers {n} 条')
    print(f'输出目录: {out}')


if __name__ == '__main__':
    main()
