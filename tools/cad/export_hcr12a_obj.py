"""Export the named HCR-12A STEP assembly groups as separate OBJ meshes.

This is an offline CAD conversion tool. It deliberately preserves each STEP
shape's placement; importing into a moving FK link requires a later local-frame
calibration rather than silently baking an unknown joint pivot into vertices.
"""
import FreeCAD as App
import Import
import Mesh
import os
import sys

if len(sys.argv) < 3:
    raise RuntimeError("usage: freecadcmd export_hcr12a_obj.py <input.stp> <output-dir>")

source = os.path.abspath(sys.argv[-2])
output_dir = os.path.abspath(sys.argv[-1])
os.makedirs(output_dir, exist_ok=True)

document = App.newDocument("Hcr12aObjExport")
Import.insert(source, document.Name)
document.recompute()

# These are the top-level motion groups carried by the supplied official STEP
# assembly: fixed base followed by six serial moving groups/tool mount.
mapping = [
    ("HCR12A_J1_HOUSING_R1", "base.obj"),
    ("HCR12A_J1_ASSY_R1_ASM", "link1.obj"),
    ("HCR12A_1ST_ARM_ASSY_R1_ASM", "link2.obj"),
    ("HCR12A_J3_J4_ASSY_R1_ASM", "link3.obj"),
    ("HCR12A_2ND_ARM_ASSY_R1_ASM", "link4.obj"),
    ("HCR12A_J5_J6_ASSY_R1_ASM", "link5.obj"),
    ("HCR12A_TOOL_IO_ASSY_R1_ASM", "link6.obj"),
]

for object_name, file_name in mapping:
    obj = document.getObject(object_name)
    if obj is None or not hasattr(obj, "Shape") or obj.Shape.isNull():
        raise RuntimeError("Missing or empty HCR assembly object: " + object_name)
    destination = os.path.join(output_dir, file_name)
    Mesh.export([obj], destination)
    print(object_name + " -> " + destination)
