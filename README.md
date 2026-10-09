# GraspLink

GraspLink is a C++17 simulator for a Hanwha HCR-12A robot arm and Robotiq 2F-85 gripper. It combines OpenGL rendering, Flecs entities, and Jolt Physics to test robot motion and pick-and-place behavior on Windows and in WebAssembly-enabled browsers.

## Features

- Forward kinematics updates the rendered robot and its physics collision proxies from joint state.
- Damped least-squares inverse kinematics supports joint-space moves and straight TCP paths.
- Pick-and-place planning combines collision-checked RRT-Connect travel with linear approach and retreat moves.
- Jolt Physics simulates the environment and dynamic objects. Grasp attachment is a simplified contact-based constraint, not a force-accurate gripper model.
- Emscripten builds provide a WebGL 2 browser version with multithreading and a separate single-threaded option.

The simulator backend is software-only. It does not connect to or control physical robot hardware. Robot acceleration and gripper mapping values are configurable simulation policies, not manufacturer-validated specifications.

## Build on Windows

Open a Visual Studio Developer PowerShell or x64 Native Tools prompt with CMake, Ninja, and MSVC available.

```powershell
cmake --preset ninja
cmake --build --preset ninja-debug
ctest --test-dir build-ninja-debug --output-on-failure
```

To build the release configuration, use `ninja-release` and `build-ninja-release` in the corresponding commands.

Run the simulator from its build directory so the copied shaders and models are available:

```powershell
cd build-ninja-debug
.\grasplink_simulator.exe
```

The GUI can open or close the gripper and start, pause, reset, or stop the pick-and-place sequence. The configured-collider view shows the simulator's collision proxies; it does not query Jolt's generated contact geometry.

## Build for the browser

Install and activate the Emscripten SDK, then configure and build the threaded target:

```powershell
emcmake cmake --preset emscripten-web
cmake --build --preset emscripten-web
```

The output is `build-emscripten-web/index.html`. It contains the browser runtime and assets in one file and can be opened directly for local use. The threaded static deployment is hosted through the Portfolio site, which supplies the cross-origin isolation setup required for `SharedArrayBuffer`.

For browsers without pthread support, build the single-threaded version:

```powershell
emcmake cmake --preset emscripten-single-thread
cmake --build --preset emscripten-single-thread
```

That output is `build-emscripten-single-thread/index.html` and runs Jolt and the logger without worker threads.

## Project structure

| Path | Purpose |
| --- | --- |
| `apps/simulator` | Application setup and main loop |
| `modules/application` | Pick-and-place mission and application flow |
| `modules/robotics` | Robot models, kinematics, and controller interfaces |
| `modules/simulation` | Flecs entities and physics synchronization |
| `modules/physics` | Jolt world and body interfaces |
| `modules/gui` | ImGui panels and simulation controls |
| `modules/viewer` | OpenGL rendering and browser graphics backend |
| `docs` | Architecture, model provenance, and design notes |

## Documentation

- [Architecture](docs/ARCHITECTURE.md)
- [Controller interface](docs/CONTROLLER_INTERFACE.md)
- [Robot and gripper specifications](docs/HCR12A_2F85_simulation_specs.md)
- [Physics and ECS integration](docs/PHYSICS_ECS_INTEGRATION.md)
- [Robot motion and grasp behavior](docs/ROBOT_MOTION_AND_GRASP.md)
- [Diagnostics](docs/DIAGNOSTICS.md)
