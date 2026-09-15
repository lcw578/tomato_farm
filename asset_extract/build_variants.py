# 12 个随机化串收番茄植株变体生成器 (seed=42+k, 可精确重现)
# 随机维度:
#   1. 主茎倾角 22~38°          2. 整株均匀缩放 0.90~1.12 (高矮/果实大小)
#   3. 果串连接方位角 0~360 (间隔>=60°)  4. 果串挂点高度 ±0.08m
#   5. 成熟度模板 (4 选 1, 自下而上递减)  6. 顶部叶幕绕茎滚转角
#   7. (世界层) 倾斜方向按行交替 ±X
# 几何/贴图 100% 作者资产; 输出 DAE + markers + 变体参数 JSON
import bpy, json, math, os, random, sys
from mathutils import Matrix, Vector

TEX = '/home/lcw/aoc_tomato_farm/asset_extract/gz_textures'
OUTROOT = '/home/lcw/aoc_tomato_farm/asset_extract/out/variants'
TEMPLATES = [['red', 'red', 'orange', 'green'],
             ['red', 'red', 'red', 'orange'],
             ['red', 'orange', 'orange', 'green'],
             ['red', 'orange', 'green', 'green']]
CYL_TOP_Z = 1.45
LOWER_Z = [0.80, 1.25]
UPPER_LOCAL = [0.25, 0.60]
TRUSS_BLEND = '/home/lcw/aoc_tomato_farm/asset_extract/unity_tomato_farm_generator/Assets/models/Tomatoes/tomato1.blend'
STRUCT_DAE = '/home/lcw/aoc_tomato_farm/asset_extract/unity_tomato_farm_generator/Assets/Plant/tomato.dae'

def mat_img(name, png, rough=0.5, spec=0.5):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = next(n for n in m.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
    tex = m.node_tree.nodes.new('ShaderNodeTexImage')
    tex.image = bpy.data.images.load(f'{TEX}/{png}')
    m.node_tree.links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])
    bsdf.inputs['Roughness'].default_value = rough
    bsdf.inputs['Specular'].default_value = spec
    return m

def build_variant(k):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    rng = random.Random(42 + k)
    theta = math.radians(rng.uniform(22, 38))          # 1) 主茎倾角
    scale = rng.uniform(0.90, 1.12)                    # 2) 整株缩放
    while True:                                        # 3) 果串方位角, 间隔>=60°
        az = sorted(rng.uniform(0, 360) for _ in range(4))
        gaps = [az[1]-az[0], az[2]-az[1], az[3]-az[2], 360-az[3]+az[0]]
        if min(gaps) >= 60:
            break
    rip = TEMPLATES[rng.randrange(4)]                  # 5) 成熟度模板
    j = [rng.uniform(-0.08, 0.08) for _ in range(4)]   # 4) 挂点高度抖动
    roll = rng.uniform(0, 360)                         # 6) 叶幕滚转

    M = {'vine': mat_img('vine', 'brn1.png', 0.42, 0.45),
         'red': mat_img('red', 'frt2.png', 0.22, 0.7),
         'org': mat_img('org', 'frt1.png', 0.22, 0.7),
         'grn': mat_img('grn', 'frt4.png', 0.24, 0.65),
         'leaf': mat_img('leaf', 'lef1.png', 0.55, 0.25),
         'blossom': mat_img('blossom', 'blo1.png', 0.5, 0.3)}

    R = Matrix.Rotation(theta, 4, 'Y')
    def vine_point(z):
        return Vector((z * math.tan(theta), 0, z))

    # ---- 0) 作者枝条导入并测量 ----
    bpy.ops.wm.collada_import(filepath=STRUCT_DAE)
    branch_objs = [ob for ob in bpy.data.objects if ob.type == 'MESH']

    def stem_center(z_lo, z_hi):
        pts = [ob.matrix_world @ v.co for ob in branch_objs
               if ob.name.startswith('Branch') for v in ob.data.vertices
               if z_lo <= (ob.matrix_world @ v.co).z <= z_hi]
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

    # ---- 1) 倾斜细藤 ----
    VINE_R = max(0.008, r_base * 0.92)
    CYL_L = CYL_TOP_Z / math.cos(theta)
    bpy.ops.mesh.primitive_cylinder_add(radius=VINE_R, depth=CYL_L, vertices=16)
    vine = bpy.context.object
    vine.matrix_world = R @ Matrix.Translation((0, 0, CYL_L / 2))
    vine.data.materials.append(M['vine'])

    # ---- 2) 枝条材质 + 滚转 + 轴向对齐 + 嫁接(嵌入6cm) ----
    for ob in branch_objs:
        n = ob.name
        if n.startswith(('Leaf', 'Blossom', 'Branch')):
            ob.data.materials.append(M['leaf' if n.startswith('Leaf')
                                       else 'blossom' if n.startswith('Blossom')
                                       else 'vine'])
    branch_objs = [ob for ob in branch_objs if ob is not vine]
    c0 = stem_center(zmin, zmin + 0.06)
    Mr = Matrix.Translation(c0) @ Matrix.Rotation(math.radians(roll), 4, 'Z') @ Matrix.Translation(-c0)
    for ob in branch_objs:
        ob.matrix_world = Mr @ ob.matrix_world
    c0 = stem_center(zmin, zmin + 0.06)
    c1 = stem_center(zmin + 0.20, zmin + 0.30)
    d = (c1 - c0).normalized()
    v_axis = Vector((math.sin(theta), 0.0, math.cos(theta)))
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
        target_z = vine_point(CYL_TOP_Z).z + local_z * math.cos(theta)
        pts = []
        for ob in branch_objs:
            if not ob.name.startswith('Branch'):
                continue
            for v in ob.data.vertices:
                wv = ob.matrix_world @ v.co
                if abs(wv.z - target_z) < 0.03:
                    pts.append(wv)
        if not pts:
            return vine_point(CYL_TOP_Z) + R @ Vector((0, 0, local_z))
        return Vector((sum(p.x for p in pts) / len(pts),
                       sum(p.y for p in pts) / len(pts),
                       sum(p.z for p in pts) / len(pts)))

    # ---- 3) 果串 ----
    def append_blend(path):
        before = set(bpy.data.objects)
        with bpy.data.libraries.load(path, link=False) as (src, dst):
            dst.objects = src.objects
        new = [ob for ob in bpy.data.objects if ob not in before]
        for ob in new:
            bpy.context.scene.collection.objects.link(ob)
        return [ob for ob in new if ob.type == 'MESH']

    def truss_kink(objs):
        cyls = [x for x in objs if x.name.startswith('Cylinder')]
        rach = max(cyls, key=lambda x: (x.matrix_world @ Vector(x.bound_box[6])).z
                                    - (x.matrix_world @ Vector(x.bound_box[0])).z)
        bb = [rach.matrix_world @ Vector(c) for c in rach.bound_box]
        return Vector(((bb[0].x + bb[6].x) / 2, (bb[0].y + bb[6].y) / 2,
                       max(b.z for b in bb)))

    def color_truss(objs, mode):
        key = {'red': 'red', 'orange': 'org', 'green': 'grn'}[mode]
        for ob in [o for o in objs if o.name.startswith('Sphere')]:
            ob.data.materials.append(M[key])
        for ob in objs:
            if ob.name.startswith('Cylinder'):
                ob.data.materials.append(M['vine'])

    truss_spheres = []   # (truss_id, ripeness, [sphere objects])
    attach_pts = [vine_point(LOWER_Z[0] + j[0]), vine_point(LOWER_Z[1] + j[1]),
                  branch_point(UPPER_LOCAL[0] + j[2]), branch_point(UPPER_LOCAL[1] + j[3])]
    for i in range(4):
        objs = append_blend(TRUSS_BLEND)
        color_truss(objs, rip[i])
        kink = truss_kink(objs)
        T = Matrix.Translation(attach_pts[i]) @ Matrix.Rotation(math.radians(az[i]), 4, 'Z') \
            @ Matrix.Translation(-kink)
        for ob in objs:
            ob.matrix_world = T @ ob.matrix_world
        spheres = [o for o in objs if o.name.startswith('Sphere')]
        cents = []
        for ob in spheres:
            c = sum((ob.matrix_world @ Vector(cor) for cor in ob.bound_box), Vector()) / 8
            cents.append([c.x, c.y, c.z])
        truss_spheres.append((i, rip[i], cents))

    # ---- 4) UV 统一 + 按材质合并 (107→6) ----
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

    # ---- 5) 整株缩放 (关于基点=原点) ----
    S = Matrix.Scale(scale, 4)
    for ob in [o for o in bpy.data.objects if o.type == 'MESH']:
        ob.matrix_world = S @ ob.matrix_world
    bpy.context.view_layer.update()

    # ---- 6) markers (合并前质心 × 整株缩放) ----
    markers = []
    for tid, ripeness, cents in truss_spheres:
        for c in cents:
            markers.append({"marker_type": "FRUIT", "truss_id": tid, "ripeness": ripeness,
                            "translation": [round(c[0]*scale, 6), round(c[1]*scale, 6), round(c[2]*scale, 6)]})

    # ---- 7) 导出 + lambert→phong 后处理 ----
    bpy.ops.object.select_all(action='SELECT')
    out = f'{OUTROOT}/v{k}'
    os.makedirs(out, exist_ok=True)
    dae = f'{out}/tomato_vine_repo_v{k}.dae'
    bpy.ops.wm.collada_export(filepath=dae, selected=True)
    import re as _re
    _s = open(dae).read()
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
    open(dae, 'w').write(_s)
    json.dump(markers, open(f'{out}/markers.json', 'w'), indent=1)
    json.dump({"theta_deg": math.degrees(theta), "scale": scale, "ripeness": rip,
               "azimuths": az, "seed": 42 + k},
              open(f'{out}/params.json', 'w'), indent=1)
    print(f'VARIANT {k}: tilt={math.degrees(theta):.1f}° scale={scale:.3f} rip={rip} az={[round(a) for a in az]} markers={len(markers)}')

os.makedirs(OUTROOT, exist_ok=True)
for k in [int(x) for x in sys.argv[sys.argv.index('--') + 1:]]:
    build_variant(k)
print('ALL DONE')
