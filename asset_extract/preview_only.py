import bpy, math
from mathutils import Vector
bpy.ops.wm.open_mainfile = None
# 只重建预览：打开刚导出的场景不现实，直接重新执行组装的前半段太慢；
# 改为独立小脚本：导入导出的 DAE 本身来渲染 —— 同时也是对导出文件的回读验证！
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.collada_import(filepath='/home/lcw/aoc_tomato_farm/asset_extract/out/tomato_plant_repo.dae')
sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'
sc.display.shading.light = 'STUDIO'
sc.display.shading.color_type = 'TEXTURE'
sc.render.resolution_x = 900
sc.render.resolution_y = 700
cam = bpy.data.cameras.new('cam'); cam_ob = bpy.data.objects.new('cam', cam)
sc.collection.objects.link(cam_ob); sc.camera = cam_ob
cam_ob.location = (1.4, -1.4, 0.85)
d = Vector((0, 0, 0.7)) - cam_ob.location          # 指向目标
cam_ob.rotation_euler = d.to_track_quat('-Z', 'Y').to_euler()
sun = bpy.data.lights.new('sun', 'SUN'); sun.energy = 3
sun_ob = bpy.data.objects.new('sun', sun)
sc.collection.objects.link(sun_ob)
sun_ob.rotation_euler = (math.radians(50), 0, math.radians(30))
sc.render.filepath = '/home/lcw/aoc_tomato_farm/asset_extract/out/preview_blender.png'
bpy.ops.render.render(write_still=True)
print('PREVIEW2 DONE, objects:', len([o for o in bpy.data.objects if o.type=="MESH"]))
