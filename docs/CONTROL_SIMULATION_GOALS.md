# Robotics Simulation Goals

## 紐⑺몴

?뚮뜑留?援ы쁽?대굹 ?뱀젙 ?섎뱶?⑥뼱? 寃고빀?섏? ?딆? ?숈씪??robotics core瑜?Simulation怨?Real Hardware?먯꽌 ?ъ궗?⑺븳??

```text
Motion Command
      ??IRobotController
      ??Backend
?쒋? Simulation
?붴? Hardware
      ??RobotState
      ??Kinematics / Physics / Viewer
```

## ?곗꽑?쒖쐞

1. **Backend 遺꾨━**
   - `IRobotController` / `IGripperController`瑜?怨듯넻 contract濡??ъ슜
   - `SimRobotController`? ?ν썑 Hardware backend瑜?遺꾨━
   - Viewer??援ъ껜?곸씤 Simulation/Hardware 援ы쁽??吏곸젒 ?섏〈?섏? ?딅뒗??

2. **Joint Angle Limit**
   - model蹂?`minPositionRadians`, `maxPositionRadians`
   - ?곸슜 ??target 寃利?   - 愿???쒗븳??踰쀬뼱??紐낅졊? ?곹깭???곸슜?섏? ?딅뒗??

3. **Velocity / Acceleration Limit**
   - model蹂?理쒕? ?띾룄 ?ъ슜
   - 紐⑺몴 媛곷룄瑜?利됱떆 ?곸슜?섏? ?딄퀬 ?쒓컙???곕씪 ?곹깭瑜?蹂??   - `velocityScale`? model 理쒕? ?띾룄?????鍮꾩쑉濡??ъ슜
   - Joint-space `accelerationScale`? ??Β룰?利앺븯吏留?愿??媛?띾룄瑜??쒗븳?섏? ?딅뒗?? Cartesian `MoveLinear`怨?`MoveLinearPath`??吏곸꽑쨌?뚯쟾 媛?띾룄 ?쒗븳???곸슜???섎굹??寃쎈줈 ?꾨줈?뚯씪???ъ슜?쒕떎.
   - ?쒖“??理쒕? 媛?띾룄 媛믪씠 ?뺤씤?섍린 ?꾩뿉???꾩쓽 ?곸닔瑜??ㅼ젣 ?ъ뼇?쇰줈 痍④툒?섏? ?딆쓬
   - Simulation??媛?띾룄 媛믪씠 ?꾩슂?섎㈃ ?쒖“???ъ뼇怨?遺꾨━?댁꽌 愿由?
4. **Fixed Control Loop**
   - Rendering FPS? ?쒖뼱 二쇨린瑜?遺꾨━
   - accumulator 湲곕컲 怨좎젙 `dt`濡?Controller瑜?Update
   - Rendering frame??`dt`瑜?Controller??吏곸젒 ?꾨떖?섏? ?딅뒗??
   - `FixedControlLoop`???뱀젙 Controller 援ы쁽??吏곸젒 ?섏〈?섏? ?딅뒗??
   - callback ?뺥깭濡?怨좎젙 timestep???꾨떖?쒕떎.
   - hard real-time scheduler媛 ?꾨땲硫?thread/sleep??吏곸젒 愿由ы븯吏 ?딅뒗??

```text
Render Frame
    ??    ??frameDelta
    ??FixedControlLoop
    ??    ?쒋? accumulator += frameDelta
    ??    ?붴? while accumulator >= fixedDt
           ??           ?쒋? callback(fixedDt)
           ?붴? accumulator -= fixedDt
```

??

```text
fixedDt = 0.004 s
frameDt = 0.010 s

tick(0.004)
tick(0.004)

remaining accumulator = 0.002 s
```

Rendering FPS媛 蹂?섎뜑?쇰룄 Controller????긽 ?숈씪??`fixedDt`瑜?諛쏅뒗??

```text
Rendering
60 FPS
144 FPS
200 FPS
...

Control
fixed dt
fixed dt
fixed dt
...
```

5. **Physics Base Integration ??implemented**
   - `modules/physics`??Jolt 珥덇린?붿? Body ?앹꽦쨌??젣쨌step留??대떦?쒕떎.
   - Viewer ECS integration? Entity ?ㅼ젙 而댄룷?뚰듃瑜??쎌뼱 Body瑜??앹꽦?섍퀬 ?섎챸???곌껐?쒕떎.
   - Render FPS? ?낅┰?곸씤 fixed step?먯꽌 Kinematic ?낅젰怨?Dynamic 寃곌낵 諛섏쁺??泥섎━?쒕떎.
   - SceneRoot ?꾨옒 Entity??World pose瑜?遺紐???뻾?щ줈 Local Transform???섎룎由곕떎.

理쒖냼 援ы쁽 踰붿쐞:

```text
PhysicsWorld
Gravity
Static Floor
Dynamic Box
Collision
Fixed Physics Step
Flecs Entity physics config -> Jolt Body
Kinematic Entity Transform -> Physics
Dynamic Physics World pose -> Entity Local Transform
```

援ъ“:

```text
Jolt Physics
    ??Position / Rotation
    ??Flecs Transform
    ??Renderer
```

Jolt ?꾩슜 ??낆? `modules/physics` ?대???寃⑸━?쒕떎. Simulation integration?먮뒗 GLM怨??꾨줈?앺듃??`PhysicsBodyHandle`留?蹂댁씠硫?Jolt `BodyID`???몄텧?섏? ?딅뒗??

6. **Robot Model / FK ??implemented**
   - HCR-12A Joint State濡쒕???base 湲곗? link pose? ToolFrame pose 怨꾩궛
   - Joint hierarchy 湲곕컲 kinematic chain ?뺤쓽
   - ToolFrame ?ы븿
   - parent-relative transform 湲곗??쇰줈 援ъ꽦
   - bind pivot ?ъ씠??李⑥씠瑜??꾩쟻 ?뚯쟾???곸슜??base-frame pose瑜?怨꾩궛?쒕떎.
   - FK ToolFrame??怨좎젙 怨듦뎄 蹂?섏쓣 ?뷀빐 Robot base 湲곗? TCP瑜?怨꾩궛?쒕떎. ToolFrame???덈뒗 Simulation Controller??紐⑤뜽 湲곕컲 `tcpPoseValid=true` ?곹깭瑜??쒓났?섎ŉ ?ㅼ젣 ?μ튂 痢≪젙媛믪씠 ?꾨땲??

```text
q1 ... q6
    ??Forward Kinematics
    ??ToolFrame Position
ToolFrame Orientation
```

寃利????

```text
Zero Pose
J1 only
J2 only
J3 only
Multiple Joint Pose
Tool Frame
```

FK 寃곌낵? GLB ToolFrame 寃곌낵瑜?鍮꾧탳?쒕떎.

7. **IK / Cartesian Motion - implemented**
   - `MovePose()`??IK 寃곌낵瑜?`MoveJoint()`???꾨떖?쒕떎. `MoveLinear()`? 寃쎈줈 ?쒕낯??IK瑜?誘몃━ 怨꾩궛?섍퀬 TCP 媛?띉룰컧???꾨줈?뚯씪濡?異붿쥌?쒕떎. ?쒓퀎? ?ㅽ뙣 ?숈옉? [濡쒕큸 ?대룞怨??뚯?](ROBOT_MOTION_AND_GRASP.md)瑜?李멸퀬?쒕떎.
   - 紐⑺몴 TCP Pose?먯꽌 Joint State 怨꾩궛
   - 珥덇린 諛⑹떇? Damped Least Squares 湲곕컲 iterative IK
   - `MovePose()`濡?IK 寃곌낵瑜?`MoveJoint()`???곌껐?섍퀬 `MoveLinear()`?먯꽌 吏곸꽑 ?꾩튂쨌理쒕떒 ?뚯쟾 寃쎈줈??IK瑜?怨꾩궛?쒕떎. ?곸꽭 怨꾩빟? [濡쒕큸 ?대룞怨??뚯?](ROBOT_MOTION_AND_GRASP.md)瑜?李멸퀬?쒕떎.

```text
Target TCP Pose
      ??IK
      ??Joint Target
      ??Joint Controller
```

寃????ぉ:

- Position Error
- Orientation Error
- Jacobian
- Joint Limit
- Singularity
- Convergence
- Maximum Iteration
- Unreachable Target

8. **Gripper Runtime Controller ???먯쑀怨듦컙 媛쒗룓 援ы쁽**
   - `IGripperController` contract? 2F-85 specification? 以鍮꾨맖
   - `SimGripperController`媛 raw ?붿껌???곗냽 `closureFraction` ?곹깭濡?媛깆떊
   - `GripperKinematics`??master/mimic Local ?뚯쟾??`GripperTransformAdapter`媛 GLB bind???곸슜
   - raw ?꾩튂???좏삎 fraction 留ㅽ븨怨?湲곕낯 master ?띾룄 0.1..1.0 rad/s???쒖“???ъ뼇???꾨땶 ?쒕??덉씠??媛??   - ?꾩옱 臾쇰━ ?묒큺???곕Ⅸ 媛쒗룓 ?뺤?? ?묒そ ?먮걹??怨좎젙 constraint ?뚯?瑜??쒓났?쒕떎. force쨌?꾨쪟쨌?묒큺 ?댄썑 媛쒕퀎 ?먭????곸쓳? 怨꾩궛?섏? ?딅뒗??

```text
positionRequest
0 ... 255
     ??GripperState.closureFraction [0,1]
     ??master linkage angle q
     ??mimic relation
     ??Gripper Joint State
     ??GLB 愿??Local ?뚯쟾 ??World 蹂????湲곗〈 7媛?Kinematic proxy
```

4 ms留덈떎 Controller 媛깆떊 ???붋룰렇由ы띁 ?먯꽭 ?곸슜 ??World 蹂??媛깆떊 ??Jolt step ??World 蹂???ш갚???쒖꽌?? GUI???명꽣?섏씠?ㅼ뿉 ?붿껌??蹂대궡硫?Entity瑜?吏곸젒 蹂寃쏀븯吏 ?딅뒗?? ?꾩옱 怨꾩빟怨?寃利앹? [洹몃━???고????ㅺ퀎](GRIPPER_RUNTIME_DESIGN.md)???뺣━?쒕떎.

Free-space 愿怨?

```text
LeftOuter  = +q
RightOuter = -q

LeftInner  = +q
RightInner = -q

LeftTip    = -q
RightTip   = +q
```

9. **Robot / Gripper Physics**
   - GLB?먯꽌 留뚮뱺 Convex Hull??蹂꾨룄 Kinematic link proxy Entity??遺숈씠??湲곕컲? 援ы쁽??   - proxy??FK??base-frame link pose瑜??곕Ⅴ硫?愿???쒖빟?대굹 ?좏겕瑜??몃뒗 articulated dynamics???꾨떂
   - Kinematic gripper collision proxy??援ы쁽??   - ?묒そ ?먮걹 ?묒큺, ?묒큺 ???뺤?, ?뚯? constraint? 臾쇱껜 ?대컲??援ы쁽?덈떎. ?ㅼ젣 ?샕룸쭏李? 媛쒕퀎 ?먭????곸쓳, Robot/Gripper 愿???숈뿭?숈? 援ы쁽?섏? ?딆븯??

```text
Robot Link
    ??Collision Shape
    ??Rigid / Kinematic Body

Gripper Finger
    ??Collision
    ??Constraint
    ??Contact
```

援ы쁽 ???

- Robot Link Collider
- Gripper Link Collider
- Joint Constraint
- Self Collision ?뺤콉
- Environment Collision
- Object Contact

10. **Physics / Grasp**
   - 2F-85??free-space mimic 愿怨꾨뒗 臾쇱껜 ?묒큺 ?꾧퉴吏 ?ъ슜
   - 臾쇱껜 ?묒큺 ?댄썑?먮뒗 ?⑥닚 `jointAngle = masterAngle * multiplier` 愿怨꾨쭔 媛뺤젣?섏? ?딅뒗??
   - ?묒큺 ?댄썑 finger adaptation? Physics / Constraint 怨꾩링?먯꽌 泥섎━?쒕떎.

```text
Finger Closing
      ??Object Contact
      ??Constraint Response
      ??Finger Adaptation
      ??Grasp State
```

援ы쁽 ???

- ContactWhileClosing
- ContactWhileOpening
- Object Detection
- Grasp
- Attach
- Detach
- Release

11. **Safety ?뺤옣**
   - software stop ?곹깭
   - watchdog
   - E-Stop state model
   - zero offset
   - invalid state 李⑤떒
   - NaN / Inf command 李⑤떒
   - command timeout

?ㅼ젣 E-Stop ?뚮줈瑜?software `Stop()`???泥댄븯吏 ?딅뒗??

## Physics Integration

```text
Flecs Entity
?쒋? Transform / Render Components
?쒋? RigidBody
?붴? Colliders
       ??       ??PhysicsSystemModule
  - config observer / private runtime binding
  - Local ??World 蹂?섍낵 fixed-step ?숆린??       ??       ??PhysicsWorld
  - PhysicsBodyHandle API
  - Jolt ?대? 援ы쁽
```

??븷:

```text
Flecs
= Entity / ECS / Application State

PhysicsWorld
= Jolt lifetime / rigid body / collision / fixed step

PhysicsSystemModule
= Flecs component ?댁꽍 / Body ?곌껐 / Transform ?숆린??
Renderer
= 寃곌낵 ?쒓컖??```

??怨꾩링? 湲곕뒫 以묐났???꾨땲?? `PhysicsWorld`??Flecs瑜?紐곕씪???섍퀬, ViewerApp? Body handle?대굹
醫뚰몴 蹂???몃??ы빆??吏곸젒 愿由ы븯吏 ?딆븘???쒕떎. `PhysicsBodyBinding`? Entity??遺숇뒗 private
runtime component?대ŉ Entity??臾쇰━ ?ㅼ젙???쒓굅????observer媛 Jolt Body瑜???젣?쒕떎.

Collider 諛??ш린??meter ?⑥쐞?대ŉ Entity scale???먮룞 怨깊븯吏 ?딅뒗?? Robot Link? Dynamic Entity??unit
scale???곌퀬, Floor???뚮뜑 GLB scale怨??낅┰??collider ?ш린瑜?紐낆떆?쒕떎. Dynamic Body 議곗긽 ?꾨옒??Dynamic
Body??binding ?앹꽦 ??嫄곕??쒕떎. Dynamic Entity??parent媛 scale/shear瑜?媛吏??寃쎌슦??吏??踰붿쐞媛 ?꾨땲??

## 援ы쁽 ?쒖꽌

```text
1. robotics/core + model/backend 遺꾨━
                    ??
2. Joint angle / velocity limit
                    ??湲곕낯 援ы쁽

3. Fixed Control Loop
                    ??250 Hz fixed step

4. Jolt Physics 理쒖냼 湲곕컲
   - PhysicsWorld
   - Gravity
   - Floor
   - Dynamic Box
   - ECS ?ㅼ젙 而댄룷?뚰듃? lifetime ?곕룞
   - Robot J1~J6 Kinematic collision proxy
                    ??湲곕낯 湲곕컲

5. Robot Model / FK
   - Kinematic Chain
   - ToolFrame
   - FK 寃利?                    ??援ы쁽

6. IK / MoveLinear

7. Gripper Runtime Controller
   - 2F-85 request
   - master q
   - free-space mimic
                    ???먯쑀怨듦컙 backend / 湲곌뎄??/ GLB쨌proxy ?곌껐
   - Force / contact stop / grasp
                    ???꾩냽 臾쇰━ ?묒뾽

8. Robot / Gripper Physics
   - Asset 硫붿떆 湲곕컲 Gripper Kinematic collider proxy
                    ??湲곕낯 湲곕컲
   - Constraint / articulated dynamics
                    ???ㅼ쓬 臾쇰━ ?뺤옣

9. Physics / Grasp
   - Contact? under-actuated adaptation
   - Attach / Detach / Release
                    ??誘멸뎄??
10. Safety
    - Watchdog
    - E-Stop State
    - Zero Offset
```
