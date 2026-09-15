# 作者 tomato1.blend → 独立果串 Gazebo 模型（与已验证整株同管线）
import bpy, json
from mathutils import Vector
TEX = '/home/lcw/aoc_tomato_farm/asset_extract/gz_textures'
OUT = '/home/lcw/aoc_tomato_farm/asset_extract/out'

bpy.ops.wm.open_mainfile(filepath='unity_tomato_farm_generator/Assets/models/Tomatoes/tomato1.blend')
def mat_img(name, png):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = next(n for n in m.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
    tex = m.node_tree.nodes.new('ShaderNodeTexImage')
    tex.image = bpy.data.images.load(f'{TEX}/{png}')
    m.node_tree.links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])
    return m
FR = [mat_img(f'fruit{i}', f'frt{i}.png') for i in range(1, 5)]
BR = mat_img('branch', 'brn1.png')
fi, spheres = 0, []
for ob in bpy.data.objects:
    if ob.type != 'MESH': continue
    if ob.name.startswith('Sphere'):
        ob.data.materials.append(FR[fi % 4]); fi += 1
        c = sum((ob.matrix_world @ Vector(cor) for cor in ob.bound_box), Vector()) / 8
        spheres.append([round(c.x,6), round(c.y,6), round(c.z,6)])
    elif ob.name.startswith('Cylinder'):
        ob.data.materials.append(BR)
bpy.ops.object.select_all(action='SELECT')
bpy.ops.wm.collada_export(filepath=f'{OUT}/tomato_truss_repo.dae', selected=True)
json.dump(spheres, open(f'{OUT}/truss_markers.json','w'), indent=1)
print('TRUSS EXPORTED, fruits:', len(spheres))
