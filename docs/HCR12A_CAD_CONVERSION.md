# HCR-12A CAD Conversion

`assets/Body HCR12A R00 3D.stp`는 source CAD 입력이며 simulator runtime이 직접 읽지 않는다.
STEP parser를 rendering application에 넣으면 build/runtime dependency와 CAD license scope가
불필요하게 커지므로, 아래의 offline workflow만 사용한다.

1. FreeCAD에서 STEP를 열고 assembly component를 base, J1~J6 link, TCP/gripper로 식별한다.
2. 각 component를 해당 joint pivot이 local origin이 되도록 배치하고, Z/Y 등 rotation axis를
   `RobotSpecification`의 axis convention과 확인한다.
3. link별 triangulated OBJ로 export한다. face는 triangle, unit은 metre, transform은 bake하지
   않고 local mesh에 유지한다.
4. Viewer의 debug axis와 zero/single-joint FK test로 parent-child 방향과 pivot을 검증한다.
5. 공식 CAD의 이용·재배포 조건을 확인한다. 조건이 불명확하면 STEP와 변환 mesh는 local asset
   으로만 두고 repository에는 변환 방법과 frame table만 기록한다.

변환 전에도 simulator는 cube fallback link를 사용해 SerialTransport, FK/IK, grasp를 실행한다. OBJ import를
추가하기 전 CAD mesh를 억지로 runtime에 로드하지 않는다.

## Portable static GLB asset

`assets/hcr12a/HCR12A_R00.glb` is the single-file preview/distribution asset
generated from the seven offline OBJ link exports. It is deliberately a
**static assembly scene**: it contains named `HCR12A_Base` through
`HCR12A_Link6_Tool` mesh nodes with their original CAD placement, but it does
not claim that their mesh origins are verified runtime joint pivots. The
kinematics module remains the source of truth for animated FK frames until the
CAD local-frame calibration table is completed.

The exporter converts FreeCAD's millimetre, Z-up coordinates to glTF metre,
Y-up coordinates through `(x, y, z) -> (x/1000, z/1000, -y/1000)`. This is a
right-handed rotation plus unit conversion, rather than a reflection, so face
winding and normal orientation remain consistent.

```powershell
python tools/cad/export_hcr12a_glb.py assets/hcr12a/links assets/hcr12a/HCR12A_R00.glb
```

Do not overwrite the source STEP with conversion output. The GLB is a derived
visual asset and must follow the original CAD licence when it is published or
redistributed.
