# MiniBroadcastCG Architecture

## 1. Project purpose

MiniBroadcastCG is a C++ and OpenGL-based broadcast graphics engine. Its eventual
goal is to composite 2D/3D graphics, text, and lower thirds over video, then add
framebuffer-based post-processing and chroma keying.

## 2. Current layer structure

```text
EngineApp
|- Window        platform window, context, events, buffer swap
|- Renderer      render-layer entry point
|- Scene         graphics that exist for the current frame
`- VideoSource   future media-input boundary

Scene
`- GraphicElement (future ImageElement, TextElement, LowerThird)
```

## 3. Class responsibilities

- `EngineApp`: coordinates initialization, updates, rendering, shutdown, and top-level lifetime.
- `Window`: owns window/context/event/swap concerns. It is already implemented and unchanged.
- `Renderer`: receives a `Scene` and owns future rendering details.
- `Scene`: will update and organize visible graphic elements.
- `GraphicElement`: common base for broadcast graphics.
- `ImageElement`, `TextElement`, `LowerThird`: placeholders for future concrete graphics.
- `VideoSource`: future boundary around video input and decoded frame access.

## 4. Why EngineApp does not know Renderer internals

`EngineApp` controls application flow, while `Renderer` controls graphics work.
This keeps future shader, texture, framebuffer, and render-pass changes inside the
rendering layer instead of spreading OpenGL details through application code.

## 5. Difference from RobotPal

RobotPal uses an ECS/Flecs-oriented design. MiniBroadcastCG deliberately uses an
explicit `Update`/`Render` flow and a small object hierarchy because its current
scope is broadcast graphics composition, not a general entity simulation system.

## 6. Data flow

```text
VideoSource -> Scene -> Renderer -> OpenGL
```

`VideoSource` will provide input frames, `Scene` will represent the graphics to
compose, and `Renderer` will translate that state into OpenGL work.

## 7. Future extension locations

- Texture and Framebuffer: `graphics/`
- Text rendering: `scene/TextElement` with renderer-owned glyph resources
- Lower Third: `scene/LowerThird`
- Chroma Key and post-processing: `graphics/effects/`
- 3D objects: `scene/` with renderer support

## 8. Intentionally excluded at this stage

- ECS
- AssetManager
- Complex event system
- Actual video decoding
- Actual rendering passes
