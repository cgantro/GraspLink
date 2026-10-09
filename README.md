# GraspLink

GraspLink is a simulator for a Hanwha HCR-12A robot arm with a Robotiq 2F-85 gripper. I built it to explore robot motion, collision checks, and pick-and-place behavior in a scene that can run on Windows or in a browser.

The simulator brings together C++17, OpenGL, Flecs, and Jolt Physics. It includes forward and inverse kinematics, joint and Cartesian motion planning, and a simple contact-based grasp model. That grasp model attaches an object after both fingers touch it; it does not calculate grip force or reproduce the real gripper’s adaptive mechanism.

GraspLink is software for simulation. It does not connect to or control a physical robot. Motion limits and gripper behavior are simulation settings, not validated operating specifications for hardware.

## Build on Windows

Use a Visual Studio Developer PowerShell or x64 Native Tools prompt with CMake, Ninja, and MSVC available.

```powershell
cmake --preset ninja
cmake --build --preset ninja-debug
ctest --test-dir build-ninja-debug --output-on-failure
```

For a release build, use the `ninja-release` configure and build presets. Start the simulator from its build directory so it can find the copied models and shaders:

```powershell
cd build-ninja-debug
.\grasplink_simulator.exe
```

The control panel can open or close the gripper and start, pause, resume, reset, or stop the pick-and-place task. The collider view shows the simplified shapes used by the simulator, rather than Jolt’s generated contact geometry.

## Build for the browser

Install and activate the Emscripten SDK, then run:

```powershell
emcmake cmake --preset emscripten-web
cmake --build --preset emscripten-web
```

The build writes `build-emscripten-web/index.html`, with the runtime and assets packaged into that file. The threaded version uses browser workers, so it needs a browser origin that permits workers and provides cross-origin isolation. The Portfolio site is configured for that deployment. Opening the threaded build as a `file://` URL does not provide those browser features.

If you need a build without pthreads, use the `emscripten-single-thread` preset instead. It writes `build-emscripten-single-thread/index.html`.

## Where things live

- `apps/simulator` starts the application and runs its main loop.
- `modules/application` coordinates the pick-and-place task.
- `modules/robotics` contains the robot model, kinematics, and controller interfaces.
- `modules/simulation` connects scene entities to physics.
- `modules/physics` wraps the Jolt world and its body handles.
- `modules/gui` contains the simulator controls.
- `modules/viewer` loads models and renders the scene.
- `docs` explains the architecture, motion behavior, model data, and diagnostics.

## Documentation

Start with the [documentation guide](docs/README.md) if you are looking for a particular topic. The most useful references are [architecture](docs/ARCHITECTURE.md), [controller interface](docs/CONTROLLER_INTERFACE.md), [robot motion and grasping](docs/ROBOT_MOTION_AND_GRASP.md), [physics and Flecs](docs/PHYSICS_ECS_INTEGRATION.md), and [diagnostics](docs/DIAGNOSTICS.md).
