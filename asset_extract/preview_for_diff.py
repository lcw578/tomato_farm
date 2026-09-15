import bpy, math, sys
from mathutils import Vector
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.collada_import(filepath=sys.argv[-1])
sc = bpy.context.scene
sc.render.engine = 'BLENDER_WORKBENCH'
sc.display.shading.light = 'STUDIO'
sc.display.shading.color_type = 'TEXTURE'
sc.render.resolution_x = 640
sc.render.resolution_y = 480
cam = bpy.data.cameras.new('c'); co = bpy.data.objects.new('c', cam)
sc.collection.objects.link(co); sc.camera = co
co.location = (3.2, -2.4, 1.5)
d = Vector((0.75, 0, 1.3)) - Vector(co.location)
co.rotation_euler = d.to_track_quat('-Z', 'Y').to_euler()
sun = bpy.data.lights.new('s', 'SUN'); sun.energy = 2.5
so = bpy.data.objects.new('s', sun); sc.collection.objects.link(so)
so.rotation_euler = (math.radians(35), math.radians(10), math.radians(20))
sc.render.filepath = sys.argv[-2]
bpy.ops.render.render(write_still=True)
print('RENDER DONE', sys.argv[-2], 'meshes:', len([o for o in bpy.data.objects if o.type=='MESH']))
