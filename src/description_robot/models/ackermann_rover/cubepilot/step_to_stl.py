import FreeCAD
import Part
import Mesh
import MeshPart
import sys

input_file = sys.argv[-2]
output_file = sys.argv[-1]

doc = FreeCAD.newDocument()

# Read STEP
shape = Part.read(input_file)

# Add to document
obj = doc.addObject("Part::Feature", "ImportedShape")
obj.Shape = shape
doc.recompute()

# Create mesh from shape (robust method)
mesh = MeshPart.meshFromShape(
    Shape=shape,
    LinearDeflection=0.1,
    AngularDeflection=0.5,
    Relative=False
)

# Export STL
Mesh.export([mesh], output_file)

print("STL exported:", output_file)
