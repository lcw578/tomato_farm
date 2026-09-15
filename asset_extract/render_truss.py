import bpy, math
from mathutils import Vector

TEX = '/home/lcw/aoc_tomato_farm/asset_extract/gz_textures'

def render_blend(path, out, view):
    bpy.ops.wm.open_mainfile(filepath=path)
    def mat_img(name, png):
        m = bpy.data.materials.new(name)
        m.use_nodes = True
        bsdf = next(n for n in m.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
        tex = m.node_tree.nodes.new('ShaderNodeTexImage')
        tex.image = bpy.data.images.load(f'{TEX}/{png}')
        m.node_tree.links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])
        return m
    FR = [mat_img(f'f{i}', f'frt{i}.png') for i in range(1, 5)]
    BR = mat_img('br', 'brn1.png')
    fi = 0
    for ob in bpy.data.objects:
        if ob.type != 'MESH': continue
        if ob.name.startswith('Sphere'):
            ob.data.materials.append(FR[fi % 4]); fi += 1
        elif ob.name.startswith('Cylinder'):
            ob.data.materials.append(BR)
    sc = bpy.context.scene
    sc.render.engine = 'BLENDER_WORKBENCH'
    sc.display.shading.light = 'STUDIO'
    sc.display.shading.color_type = 'TEXTURE'
    sc.render.resolution_x = 760
    sc.render.resolution_y = 1000
    lo, hi = Vector((-0.13, -0.14, 0.84)), Vector((0.03, 0.18, 1.17))
    ctr = (lo + hi) / 2
    cam = bpy.data.cameras.new('c'); co = bpy.data.objects.new('c', cam)
    sc.collection.objects.link(co); sc.camera = co
    if view == 'front':   co.location = (ctr.x, -0.75, ctr.z)
    elif view == 'side':  co.location = (0.75, ctr.y, ctr.z)
    else:                 co.location = (0.55, -0.55, ctr.z + 0.1)
    d = ctr - co.location
    co.rotation_euler = d.to_track_quat('-Z', 'Y').to_euler()
    sun = bpy.data.lights.new('s', 'SUN'); sun.energy = 2.5
    so = bpy.data.objects.new('s', sun); sc.collection.objects.link(so)
    so.rotation_euler = (math.radians(35), math.radians(15), 0)
    sc.render.filepath = out
    bpy.ops.render.render(write_still=True)
    print('DONE', out)

B = 'unity_tomato_farm_generator/Assets/models/Tomatoes/'
render_blend(B+'tomato1.blend',  '/home/lcw/aoc_tomato_farm/asset_extract/out/author_truss_tomato1_front.png', 'front')
render_blend(B+'tomato1.blend',  '/home/lcw/aoc_tomato_farm/asset_extract/out/author_truss_tomato1_iso.png', 'iso')
render_blend(B+'tomatoT2.blend', '/home/lcw/aoc_tomato_farm/asset_extract/out/author_truss_tomatoT2_front.png', 'front')
