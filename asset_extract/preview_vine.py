import bpy, math
from mathutils import Vector
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.collada_import(filepath='/home/lcw/aoc_tomato_farm/asset_extract/out/tomato_vine_repo.dae')
sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'
sc.display.shading.light = 'STUDIO'
sc.display.shading.color_type = 'TEXTURE'
sc.render.resolution_x = 700
sc.render.resolution_y = 1100

def shot(name, cam_loc, target):
    cam = bpy.data.cameras.new('c'); co = bpy.data.objects.new('c', cam)
    sc.collection.objects.link(co); sc.camera = co
    co.location = cam_loc
    d = Vector(target) - Vector(cam_loc)
    co.rotation_euler = d.to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = f'/home/lcw/aoc_tomato_farm/asset_extract/out/{name}.png'
    bpy.ops.render.render(write_still=True)
    print('SHOT', name)

sun = bpy.data.lights.new('s', 'SUN'); sun.energy = 2.5
so = bpy.data.objects.new('s', sun); sc.collection.objects.link(so)
so.rotation_euler = (math.radians(35), math.radians(10), math.radians(20))
shot('vine_preview_full', (3.2, -2.4, 1.5), (0.75, 0, 1.3))
shot('vine_preview_low',  (2.0, -1.7, 1.05), (0.55, 0, 0.9))
shot('vine_preview_graft', (2.1, -1.4, 1.85), (0.88, 0, 1.52))
