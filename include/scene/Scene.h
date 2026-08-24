#pragma once

// Represents the graphics scheduled for the current broadcast frame.
// Element ownership and traversal will be added when concrete elements are rendered.
class Scene {
public:
    void Update(float deltaTime);
};
