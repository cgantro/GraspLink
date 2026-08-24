#pragma once

#include "scene/GraphicElement.h"

// Placeholder for an image-based overlay such as a PNG logo.
class ImageElement final : public GraphicElement {
public:
    void Update(float deltaTime) override;
};
