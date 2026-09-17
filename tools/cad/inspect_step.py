"""Print FreeCAD's imported STEP object tree without modifying the source CAD."""
import FreeCAD as App
import Import
import os
import sys

if len(sys.argv) < 2:
    raise RuntimeError("usage: freecadcmd inspect_step.py <input.stp>")

# FreeCAD retains its executable/script arguments in sys.argv on some builds;
# the final argument is the explicit CAD input in both invocation forms.
source = os.path.abspath(sys.argv[-1])
document = App.newDocument("Hcr12aInspection")
Import.insert(source, document.Name)
document.recompute()

for obj in document.Objects:
    shape = getattr(obj, "Shape", None)
    volume = shape.Volume if shape and not shape.isNull() else 0.0
    print("{0}\t{1}\t{2}\t{3:.3f}".format(obj.Name, obj.Label, obj.TypeId, volume))
