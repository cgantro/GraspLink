#pragma once

#include "scene/GraphicElement.h"

// Placeholder for FreeType-backed text rendered by a future renderer implementation.
class TextElement final : public GraphicElement {
public:
    void Update(float deltaTime) override;
};
