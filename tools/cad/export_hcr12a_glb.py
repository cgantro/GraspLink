"""Create one portable glTF binary asset from the HCR-12A link OBJ exports.

The supplied STEP assembly is in millimetres and uses FreeCAD's Z-up frame.
glTF uses metres and Y-up.  Each vertex is therefore transformed by
``(x_mm, y_mm, z_mm) -> (x_m, z_m, -y_m)``.  This is a proper rotation plus a
millimetre-to-metre scale, so it preserves the model's right-handed geometry
instead of mirroring it.

The output is one ``.glb`` file containing seven named meshes.  The mesh nodes
retain their imported assembly placement; no unverified FK joint-pivot
calibration is baked into the asset.  This makes the result useful as a static
robot model now, while preserving an honest boundary before articulated runtime
rendering is introduced.

Only the OBJ subset exported by FreeCAD is consumed: ``v`` positions and
triangular ``f vertex//normal`` records.  Face normals are recomputed from
positions and accumulated per vertex, which avoids exploding the vertex count
merely because FreeCAD emitted a normal index for each triangle corner.
"""

import argparse
import array
import json
import math
import os
import struct


LINKS = (
    ("HCR12A_Base", "base.obj"),
    ("HCR12A_Link1", "link1.obj"),
    ("HCR12A_Link2", "link2.obj"),
    ("HCR12A_Link3", "link3.obj"),
    ("HCR12A_Link4", "link4.obj"),
    ("HCR12A_Link5", "link5.obj"),
    ("HCR12A_Link6_Tool", "link6.obj"),
)


def append_aligned(buffer, payload):
    """Append a GLB buffer view at a four-byte aligned byte offset."""
    while len(buffer) % 4:
        buffer.append(0)
    offset = len(buffer)
    buffer.extend(payload)
    return offset


def parse_obj(path):
    """Read FreeCAD OBJ geometry and return metres/Y-up positions plus indices."""
    positions = array.array("f")
    indices = array.array("I")
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]

    with open(path, "rt", encoding="utf-8", errors="strict") as source:
        for line in source:
            if line.startswith("v "):
                parts = line.split()
                # CAD mm, Z-up -> glTF m, Y-up.  The minus sign implements a
                # rotation, not a reflection, and keeps the face winding valid.
                vertex = (float(parts[1]) / 1000.0,
                          float(parts[3]) / 1000.0,
                          -float(parts[2]) / 1000.0)
                positions.extend(vertex)
                for axis, value in enumerate(vertex):
                    minimum[axis] = min(minimum[axis], value)
                    maximum[axis] = max(maximum[axis], value)
            elif line.startswith("f "):
                vertices = line.split()[1:]
                if len(vertices) != 3:
                    raise ValueError("Only triangulated FreeCAD OBJ faces are supported: " + path)
                for vertex in vertices:
                    # OBJ indices are one-based; the FreeCAD exporter uses
                    # positive vertex//normal references.
                    position_index = int(vertex.split("/", 1)[0]) - 1
                    indices.append(position_index)

    if not positions or not indices:
        raise ValueError("OBJ has no triangle geometry: " + path)
    return positions, indices, minimum, maximum


def calculate_vertex_normals(positions, indices):
    """Produce smooth unit normals by summing area-weighted face normals."""
    normals = array.array("f", [0.0]) * len(positions)
    for offset in range(0, len(indices), 3):
        a, b, c = (indices[offset] * 3, indices[offset + 1] * 3, indices[offset + 2] * 3)
        abx = positions[b] - positions[a]
        aby = positions[b + 1] - positions[a + 1]
        abz = positions[b + 2] - positions[a + 2]
        acx = positions[c] - positions[a]
        acy = positions[c + 1] - positions[a + 1]
        acz = positions[c + 2] - positions[a + 2]
        nx = aby * acz - abz * acy
        ny = abz * acx - abx * acz
        nz = abx * acy - aby * acx
        for index in (a, b, c):
            normals[index] += nx
            normals[index + 1] += ny
            normals[index + 2] += nz

    for offset in range(0, len(normals), 3):
        length = math.sqrt(normals[offset] ** 2 + normals[offset + 1] ** 2 + normals[offset + 2] ** 2)
        if length > 0.0:
            normals[offset] /= length
            normals[offset + 1] /= length
            normals[offset + 2] /= length
        else:
            normals[offset + 2] = 1.0
    return normals


def bytes_of(values):
    """Return little-endian binary payload regardless of host byte order."""
    values = array.array(values.typecode, values)
    if values.itemsize > 1 and os.sys.byteorder != "little":
        values.byteswap()
    return values.tobytes()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_dir", help="directory containing base.obj and link1.obj ... link6.obj")
    parser.add_argument("output_glb", help="destination .glb path")
    args = parser.parse_args()

    binary = bytearray()
    gltf = {
        "asset": {"version": "2.0", "generator": "GraspLink FreeCAD OBJ to GLB exporter"},
        "scene": 0,
        "scenes": [{"name": "HCR-12A R00", "nodes": []}],
        "nodes": [],
        "meshes": [],
        "materials": [{
            "name": "HCR-12A industrial orange",
            "pbrMetallicRoughness": {"baseColorFactor": [0.85, 0.16, 0.03, 1.0], "metallicFactor": 0.55, "roughnessFactor": 0.42},
        }],
        "buffers": [],
        "bufferViews": [],
        "accessors": [],
    }

    for node_index, (name, file_name) in enumerate(LINKS):
        source_path = os.path.join(args.input_dir, file_name)
        positions, indices, minimum, maximum = parse_obj(source_path)
        normals = calculate_vertex_normals(positions, indices)

        position_offset = append_aligned(binary, bytes_of(positions))
        normal_offset = append_aligned(binary, bytes_of(normals))
        index_offset = append_aligned(binary, bytes_of(indices))
        position_view = len(gltf["bufferViews"])
        gltf["bufferViews"].append({"buffer": 0, "byteOffset": position_offset, "byteLength": len(positions) * 4, "target": 34962})
        normal_view = len(gltf["bufferViews"])
        gltf["bufferViews"].append({"buffer": 0, "byteOffset": normal_offset, "byteLength": len(normals) * 4, "target": 34962})
        index_view = len(gltf["bufferViews"])
        gltf["bufferViews"].append({"buffer": 0, "byteOffset": index_offset, "byteLength": len(indices) * 4, "target": 34963})
        position_accessor = len(gltf["accessors"])
        gltf["accessors"].append({"bufferView": position_view, "componentType": 5126, "count": len(positions) // 3, "type": "VEC3", "min": minimum, "max": maximum})
        normal_accessor = len(gltf["accessors"])
        gltf["accessors"].append({"bufferView": normal_view, "componentType": 5126, "count": len(normals) // 3, "type": "VEC3"})
        index_accessor = len(gltf["accessors"])
        gltf["accessors"].append({"bufferView": index_view, "componentType": 5125, "count": len(indices), "type": "SCALAR"})
        gltf["meshes"].append({"name": name, "primitives": [{"attributes": {"POSITION": position_accessor, "NORMAL": normal_accessor}, "indices": index_accessor, "material": 0}]})
        gltf["nodes"].append({"name": name, "mesh": node_index})
        gltf["scenes"][0]["nodes"].append(node_index)
        print("packed", file_name, "vertices=", len(positions) // 3, "triangles=", len(indices) // 3)

    gltf["buffers"].append({"byteLength": len(binary)})
    document = json.dumps(gltf, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    while len(document) % 4:
        document += b" "
    while len(binary) % 4:
        binary.append(0)
    total_length = 12 + 8 + len(document) + 8 + len(binary)
    os.makedirs(os.path.dirname(os.path.abspath(args.output_glb)), exist_ok=True)
    with open(args.output_glb, "wb") as destination:
        destination.write(struct.pack("<III", 0x46546C67, 2, total_length))
        destination.write(struct.pack("<II", len(document), 0x4E4F534A))
        destination.write(document)
        destination.write(struct.pack("<II", len(binary), 0x004E4942))
        destination.write(binary)
    print("wrote", args.output_glb, "bytes=", total_length)


if __name__ == "__main__":
    main()
