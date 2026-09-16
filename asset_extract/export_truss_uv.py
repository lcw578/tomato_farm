# 从 tomato1.blend 导出带 UV/法线的果串网格（fruits = Sphere.*, stems = Cylinder.*）
# 坐标为株体坐标（z 0.85~1.16），与旧 fruits.dae/stems.dae 一致
import bpy, os

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = f'{BASE}/asset_extract/unity_tomato_farm_generator/Assets/models/Tomatoes/tomato1.blend'
OUT = f'{BASE}/asset_extract/out/truss_uv'

bpy.ops.wm.open_mainfile(filepath=SRC)

def group_and_bake(prefix, name):
    objs = [ob for ob in bpy.data.objects if ob.type == 'MESH' and ob.name.startswith(prefix)]
    bpy.ops.object.select_all(action='DESELECT')
    for ob in objs:
        ob.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    # 先把每个对象的世界变换烘进顶点，再合并，得到与旧文件一致的烘焙坐标
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    if len(objs) > 1:
        bpy.ops.object.join()
    ob = bpy.context.view_layer.objects.active
    ob.name = name
    ob.data.name = name
    print('GROUPED', name, '| objects:', len(objs), '| verts:', len(ob.data.vertices),
          '| UV:', [u.name for u in ob.data.uv_layers])
    return ob

fruits = group_and_bake('Sphere', 'fruits')
stems = group_and_bake('Cylinder', 'stems')

os.makedirs(OUT, exist_ok=True)
for ob, path in [(fruits, f'{OUT}/fruits.dae'), (stems, f'{OUT}/stems.dae')]:
    bpy.ops.object.select_all(action='DESELECT')
    ob.select_set(True)
    bpy.context.view_layer.objects.active = ob
    bpy.ops.wm.collada_export(filepath=path, selected=True, apply_modifiers=True)
    print('WROTE', path)
