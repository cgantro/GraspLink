#pragma once

// Common base for visible broadcast graphics.
// Position, scale, opacity, visibility, and render order belong here when elements gain behavior.
class GraphicElement {
public:
    virtual ~GraphicElement() = default;
    virtual void Update(float deltaTime);
};
