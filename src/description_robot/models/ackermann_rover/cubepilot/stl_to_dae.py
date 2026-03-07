import bpy
import sys

input_file = sys.argv[-2]
output_file = sys.argv[-1]

# Clear scene
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

# Import STL
bpy.ops.import_mesh.stl(filepath=input_file)

obj = bpy.context.selected_objects[0]

# Scale mm → meters
obj.scale = (0.001, 0.001, 0.001)

# Apply transform
bpy.context.view_layer.objects.active = obj
bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

# Fix normals
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.normals_make_consistent(inside=False)
bpy.ops.object.mode_set(mode='OBJECT')

# Export DAE
bpy.ops.wm.collada_export(
    filepath=output_file,
    apply_modifiers=True,
)

print("DAE exported:", output_file)
