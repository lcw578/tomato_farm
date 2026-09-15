# 单株串收番茄 v2：
#   下半段: 倾斜细藤(30°) + 3 条串(红/红/半) —— 打叶后光秆, 果实裸露
#   上半段: 作者完整结果枝 (tomato.dae: Branch1+叶+花, 连接关系零改动) + 全青串
# 几何/贴图 100% 作者资产 (tomato1.blend 串, tomato.dae 枝叶花, AG15 扫描贴图)
#   红果=frt2(深红), 转色=frt1(橙), 青果=frt4_green(frt4 调色衍生)
import bpy, json, math, os
from mathutils import Matrix, Vector

TEX = '/home/lcw/aoc_tomato_farm/asset_extract/gz_textures'
OUT = '/home/lcw/aoc_tomato_farm/asset_extract/out'
TILT = math.radians(30.0)
CYL_TOP_Z = 1.45                     # 细藤顶端(世界z), 之上接作者枝条
CYL_L = CYL_TOP_Z / math.cos(TILT)   # 斜圆柱实际长度
LOWER_Z = [0.80, 1.25]               # 下半段串挂点: 红, 红
UPPER_LOCAL = [0.25, 0.60]           # 枝条上的串挂点(枝条局部z): 半, 青
RIPENESS = ['red', 'red', 'orange', 'green']
TRUSS_AZ = [15, 130, 245, 335]

bpy.ops.wm.read_factory_settings(use_empty=True)

def mat_img(name, png, rough=0.5, spec=0.5):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = next(n for n in m.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
    tex = m.node_tree.nodes.new('ShaderNodeTexImage')
    tex.image = bpy.data.images.load(f'{TEX}/{png}')
    m.node_tree.links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])
    # 番茄蜡质高光: 对应作者 Unity 材质 smoothness≈0.5 的观感, Collada 导出为 phong specular
    bsdf.inputs['Roughness'].default_value = rough
    bsdf.inputs['Specular'].default_value = spec
    return m

M = {'vine': mat_img('vine', 'brn1.png', rough=0.42, spec=0.45),
     'red': mat_img('red', 'frt2.png', rough=0.22, spec=0.7),
     'org': mat_img('org', 'frt1.png', rough=0.22, spec=0.7),
     'grn': mat_img('grn', 'frt4.png', rough=0.24, spec=0.65),
     'leaf': mat_img('leaf', 'lef1.png', rough=0.55, spec=0.25),
     'blossom': mat_img('blossom', 'blo1.png', rough=0.5, spec=0.3)}

# ---- 0) 先导入作者枝条并测量(供藤半径与轴向对齐) ----
bpy.ops.wm.collada_import(filepath='unity_tomato_farm_generator/Assets/Plant/tomato.dae')
branch_objs = [ob for ob in bpy.data.objects if ob.type == 'MESH']
branch_mesh = next(ob for ob in branch_objs if ob.name.startswith('Branch'))

def stem_center(z_lo, z_hi):
    pts = []
    for ob in branch_objs:
        if not ob.name.startswith('Branch'):
            continue
        for v in ob.data.vertices:
            w = ob.matrix_world @ v.co
            if z_lo <= w.z <= z_hi:
                pts.append(w)
    if not pts:
        return Vector((0, 0, z_lo))
    return Vector((sum(p.x for p in pts)/len(pts), sum(p.y for p in pts)/len(pts),
                   sum(p.z for p in pts)/len(pts)))

zmin = min((ob.matrix_world @ v.co).z for ob in branch_objs
           if ob.name.startswith('Branch') for v in ob.data.vertices)
c0 = stem_center(zmin, zmin + 0.06)
c1 = stem_center(zmin + 0.20, zmin + 0.30)
r_base = max(math.hypot((ob.matrix_world @ v.co).x - c0.x,
                        (ob.matrix_world @ v.co).y - c0.y)
             for ob in branch_objs if ob.name.startswith('Branch')
             for v in ob.data.vertices
             if zmin <= (ob.matrix_world @ v.co).z <= zmin + 0.05)
print(f'MEASURED branch base radius={r_base:.4f}')

# ---- 1) 下半段倾斜细藤(半径与枝条基部匹配) ----
VINE_R = max(0.008, r_base * 0.92)
bpy.ops.mesh.primitive_cylinder_add(radius=VINE_R, depth=CYL_L, vertices=16)
vine = bpy.context.object
R = Matrix.Rotation(TILT, 4, 'Y')
vine.matrix_world = R @ Matrix.Translation((0, 0, CYL_L / 2))
vine.data.materials.append(M['vine'])

def vine_point(z):
    """细藤上世界高度 z 处的挂点"""
    return Vector((z * math.tan(TILT), 0, z))

# ---- 2) 作者完整结果枝(整组)接到藤顶, 沿倾斜方向延续 ----
for ob in branch_objs:
    n = ob.name
    if n.startswith(('Leaf', 'Blossom', 'Branch')):
        ob.data.materials.append(M['leaf' if n.startswith('Leaf')
                                   else 'blossom' if n.startswith('Blossom')
                                   else 'vine'])
# 嫁接: 三维旋转把枝条底段轴向精确对到藤轴方向, 基部嵌入藤顶 6cm 藏住接缝
branch_objs = [ob for ob in branch_objs if ob is not vine]
c0 = stem_center(zmin, zmin + 0.06)
c1 = stem_center(zmin + 0.20, zmin + 0.30)
d = (c1 - c0).normalized()
v_axis = Vector((math.sin(TILT), 0.0, math.cos(TILT)))
q = d.rotation_difference(v_axis)
Mc = Matrix.Translation(c0) @ q.to_matrix().to_4x4() @ Matrix.Translation(-c0)
for ob in branch_objs:
    ob.matrix_world = Mc @ ob.matrix_world
bpy.context.view_layer.update()
c0 = stem_center(zmin, zmin + 0.06)
Mbr = Matrix.Translation(vine_point(CYL_TOP_Z - 0.06)) @ R @ Matrix.Translation(-c0)
for ob in branch_objs:
    ob.matrix_world = Mbr @ ob.matrix_world
bpy.context.view_layer.update()

def branch_point(local_z):
    """枝条局部高度 local_z 处的主枝表面点(世界坐标)"""
    target_z = vine_point(CYL_TOP_Z).z + local_z * math.cos(TILT)
    pts = []
    for ob in branch_objs:
        if not ob.name.startswith('Branch'):
            continue
        for v in ob.data.vertices:
            w = ob.matrix_world @ v.co
            if abs(w.z - target_z) < 0.03:
                pts.append(w)
    if not pts:
        return vine_point(CYL_TOP_Z) + R @ Vector((0, 0, local_z))
    return Vector((sum(p.x for p in pts) / len(pts),
                   sum(p.y for p in pts) / len(pts),
                   sum(p.z for p in pts) / len(pts)))

# ---- 3) 追加果串并挂到藤/枝上 ----
def append_blend(path):
    before = set(bpy.data.objects)
    with bpy.data.libraries.load(path, link=False) as (src, dst):
        dst.objects = src.objects
    new = [ob for ob in bpy.data.objects if ob not in before]
    for ob in new:
        bpy.context.scene.collection.objects.link(ob)
    return [ob for ob in new if ob.type == 'MESH']

def truss_kink(objs):
    """穗轴顶端锚点 = z 跨度最大的圆柱网格的 bbox 顶面中心"""
    cyls = [x for x in objs if x.name.startswith('Cylinder')]
    rach = max(cyls, key=lambda x: (x.matrix_world @ Vector(x.bound_box[6])).z
                                - (x.matrix_world @ Vector(x.bound_box[0])).z)
    bb = [rach.matrix_world @ Vector(c) for c in rach.bound_box]
    return Vector(((bb[0].x + bb[6].x) / 2, (bb[0].y + bb[6].y) / 2,
                   max(b.z for b in bb)))

def color_truss(objs, mode):
    # 整串单色(作者 Unity prefab 同方案): red=frt2, orange=frt3, green=frt4_green
    key = {'red': 'red', 'orange': 'org', 'half': 'org', 'green': 'grn'}[mode]
    for ob in [o for o in objs if o.name.startswith('Sphere')]:
        ob.data.materials.append(M[key])
    for ob in objs:
        if ob.name.startswith('Cylinder'):
            ob.data.materials.append(M['vine'])

markers = []
attach_pts = [vine_point(z) for z in LOWER_Z] + \
             [branch_point(lz) for lz in UPPER_LOCAL]
for i, (rip, az, apt) in enumerate(zip(RIPENESS, TRUSS_AZ, attach_pts)):
    objs = append_blend('unity_tomato_farm_generator/Assets/models/Tomatoes/tomato1.blend')
    color_truss(objs, rip)
    kink = truss_kink(objs)
    T = Matrix.Translation(apt) @ Matrix.Rotation(math.radians(az), 4, 'Z') \
        @ Matrix.Translation(-kink)
    for ob in objs:
        ob.matrix_world = T @ ob.matrix_world
    for ob in objs:
        if ob.name.startswith('Sphere'):
            c = sum((ob.matrix_world @ Vector(cor) for cor in ob.bound_box), Vector()) / 8
            markers.append({"marker_type": "FRUIT", "truss_id": i, "ripeness": rip,
                            "translation": [round(c.x, 6), round(c.y, 6), round(c.z, 6)]})

json.dump(markers, open(f'{OUT}/vine_markers.json', 'w'), indent=1)

# ---- 4) 按材质合并子网格(107→6): draw call 优化, 几何/UV/法线/贴图零变化 ----
# 先统一 UV 层名: 作者 DAE 的 UV 层按几何 ID 命名(各不相同), 不统一则 join 产生多 UV 层导致贴图引用错乱
for ob in [o for o in bpy.data.objects if o.type == 'MESH']:
    uvs = ob.data.uv_layers
    while len(uvs) > 1:
        uvs.remove(uvs[-1])
    if len(uvs):
        uvs[0].name = 'UVMap'

from collections import defaultdict
groups = defaultdict(list)
for ob in [o for o in bpy.data.objects if o.type == 'MESH']:
    groups[ob.data.materials[0].name].append(ob)
for mat_name, objs in groups.items():
    bpy.ops.object.select_all(action='DESELECT')
    for ob in objs:
        ob.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    bpy.ops.object.join()
print('MERGED submeshes:', sum(len(v) for v in groups.values()), '->',
      len([o for o in bpy.data.objects if o.type == 'MESH']),
      '| 分组:', {k: len(v) for k, v in groups.items()})

bpy.ops.object.select_all(action='SELECT')
bpy.ops.wm.collada_export(filepath=f'{OUT}/tomato_vine_repo.dae', selected=True)

# ---- 后处理: Blender DAE 导出只写 lambert(高光被丢弃) → 果实材质改为 phong 注入蜡质高光 ----
import re as _re
_dae = f'{OUT}/tomato_vine_repo.dae'
_s = open(_dae).read()
SPEC = {'red-effect': (45, 0.55), 'org-effect': (45, 0.55), 'grn-effect': (40, 0.5),
        'vine-effect': (15, 0.3)}
def to_phong(seg, shin, sp):
    seg = seg.replace('<lambert>', '<phong>').replace('</lambert>', '</phong>')
    inject = (f'<specular><color sid="specular">{sp} {sp} {sp} 1</color></specular>'
              f'<shininess><float sid="shininess">{shin}</float></shininess>')
    return _re.sub(r'(</diffuse>)', r'\1' + inject, seg, count=1)
for eff, (shin, sp) in SPEC.items():
    pat = _re.compile(r'<effect id="' + eff + r'".*?</effect>', _re.S)
    m = pat.search(_s)
    if m and '<lambert>' in m.group(0):
        _s = _s[:m.start()] + to_phong(m.group(0), shin, sp) + _s[m.end():]
open(_dae, 'w').write(_s)
print('VINE V2 EXPORTED markers:', len(markers), '| phong effects:', _s.count('<phong>'))
