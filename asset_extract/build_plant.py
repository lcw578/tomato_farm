# 组装作者资产为完整串收番茄植株（几何/UV 100% 来自仓库资产）
#   结构: unity_tomato_farm_generator/Assets/Plant/tomato.dae (真主茎/真叶/真花)
#   果串: unity_tomato_farm_generator/Assets/models/Tomatoes/tomato1.blend, tomatoT2.blend
#         (弧形主果梗 + 12 根果梗 + 12 颗果实, 自带 UV, 已存于株体坐标 z0.85~1.16)
# 本脚本只做: 材质指定(复用作者 AG15 贴图) + 果串摆放(组装) + 导出/真值
import bpy, json, math, sys, os
from mathutils import Matrix, Vector

TEX = '/home/lcw/aoc_tomato_farm/asset_extract/gz_textures'
OUT = '/home/lcw/aoc_tomato_farm/asset_extract/out'

bpy.ops.wm.read_factory_settings(use_empty=True)

def mat_img(name, png, use_alpha):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes['Principled BSDF']
    tex = m.node_tree.nodes.new('ShaderNodeTexImage')
    tex.image = bpy.data.images.load(f'{TEX}/{png}')
    m.node_tree.links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])
    if use_alpha:
        m.node_tree.links.new(tex.outputs['Alpha'], bsdf.inputs['Alpha'])
        m.blend_method = 'CLIP'
        m.alpha_threshold = 0.5
    return m

M = {
    'branch':  mat_img('branch',  'brn1.png', False),
    'leaf1':   mat_img('leaf1',   'lef1.png', True),
    'leaf2':   mat_img('leaf2',   'lef2.png', True),
    'blossom': mat_img('blossom', 'blo1.png', True),
    'fruit1':  mat_img('fruit1',  'frt1.png', False),
    'fruit2':  mat_img('fruit2',  'frt2.png', False),
    'fruit3':  mat_img('fruit3',  'frt3.png', False),
    'fruit4':  mat_img('fruit4',  'frt4.png', False),
}

# ---- 1) 植株结构（作者 DAE：Branch1 / Leaf1 / Leaf2 / Blossom1~3）----
bpy.ops.wm.collada_import(filepath='unity_tomato_farm_generator/Assets/Plant/tomato.dae')
for ob in bpy.data.objects:
    if ob.type != 'MESH':
        continue
    n = ob.name
    if n.startswith('Branch'):
        ob.data.materials.append(M['branch'])
    elif n == 'Leaf1':
        ob.data.materials.append(M['leaf1'])
    elif n == 'Leaf2':
        ob.data.materials.append(M['leaf2'])
    elif n.startswith('Blossom'):
        ob.data.materials.append(M['blossom'])

# ---- 2) 追加两条作者果串 blend（自带株体坐标）----
def append_blend(path):
    before = set(bpy.data.objects)
    with bpy.data.libraries.load(path, link=False) as (src, dst):
        dst.objects = src.objects
    new = [ob for ob in bpy.data.objects if ob not in before]
    for ob in new:
        bpy.context.scene.collection.objects.link(ob)
    return [ob for ob in new if ob.type == 'MESH']

def assign_truss(meshes, ripeness_cycle):
    for ob in meshes:
        if ob.name.startswith('Cylinder'):    # 主果梗 + 各果实小果梗
            ob.data.materials.append(M['branch'])
        elif ob.name.startswith('Sphere'):    # 果实：混挂不同成熟度（对照真实照片）
            idx = int(ob.name.split('.')[-1]) % len(ripeness_cycle)
            ob.data.materials.append(M[ripeness_cycle[idx]])
    return meshes

RIP_A = ['fruit1', 'fruit3', 'fruit2', 'fruit1', 'fruit4', 'fruit2']
RIP_B = ['fruit2', 'fruit1', 'fruit4', 'fruit3', 'fruit1', 'fruit2']

trussA = assign_truss(append_blend('unity_tomato_farm_generator/Assets/models/Tomatoes/tomato1.blend'), RIP_A)
trussB = assign_truss(append_blend('unity_tomato_farm_generator/Assets/models/Tomatoes/tomatoT2.blend'), RIP_B)

# ---- 3) 果串摆放（组装：绕株轴错开方位，其中两条下移挂中/低层）----
def rotate_group(objs, angle_deg, z_shift=0.0):
    R = Matrix.Rotation(math.radians(angle_deg), 4, 'Z')
    T = Matrix.Translation((0, 0, z_shift))
    for ob in objs:
        ob.matrix_world = T @ R @ ob.matrix_world

rotate_group(trussB, 120, -0.30)   # 中层
trussC = []
for ob in trussA:                  # 下层复制品
    d = ob.copy(); d.data = ob.data.copy()
    bpy.context.scene.collection.objects.link(d)
    trussC.append(d)
rotate_group(trussC, 240, -0.52)

bpy.context.view_layer.update()

# ---- 4) 果实中心真值 markers（世界坐标, 按 truss 分组）----
def sphere_centroids(objs):
    out = []
    for ob in objs:
        if ob.name.startswith('Sphere'):
            c = sum((ob.matrix_world @ Vector(cor) for cor in ob.bound_box), Vector()) / 8
            out.append(c)
    return out

markers = []
for tid, grp in enumerate([trussA, trussB, trussC]):
    for c in sphere_centroids(grp):
        markers.append({"marker_type": "FRUIT", "truss_id": tid,
                        "translation": [round(c.x, 6), round(c.y, 6), round(c.z, 6)]})
os.makedirs(OUT, exist_ok=True)
json.dump(markers, open(f'{OUT}/markers.json', 'w'), indent=1)

# ---- 5) 导出 DAE（带材质与 UV）----
bpy.ops.object.select_all(action='SELECT')
bpy.ops.wm.collada_export(filepath=f'{OUT}/tomato_plant_repo.dae',
                          selected=True,
                          apply_modifiers=True)
print('EXPORTED', f'{OUT}/tomato_plant_repo.dae', 'markers:', len(markers))

# ---- 6) Blender 内置渲染预览（快速目检）----
sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'
sc.display.shading.light = 'STUDIO'
sc.display.shading.color_type = 'TEXTURE'
sc.render.resolution_x = 900
sc.render.resolution_y = 700
cam = bpy.data.cameras.new('cam'); cam_ob = bpy.data.objects.new('cam', cam)
sc.collection.objects.link(cam_ob); sc.camera = cam_ob
cam_ob.location = (1.4, -1.4, 0.85)
d = cam_ob.location - Vector((0, 0, 0.7))
cam_ob.rotation_euler = d.to_track_quat('-Z', 'Y').to_euler()
sun = bpy.data.lights.new('sun', 'SUN'); sun.energy = 3
sun_ob = bpy.data.objects.new('sun', sun)
sc.collection.objects.link(sun_ob)
sun_ob.rotation_euler = (math.radians(50), 0, math.radians(30))
sc.render.filepath = f'{OUT}/preview_blender.png'
bpy.ops.render.render(write_still=True)
print('PREVIEW DONE')
