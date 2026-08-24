#pragma once

#include <string>

// Opaque frame type reserved for a future media backend such as OpenCV.
struct VideoFrame;

// Defines the application-facing boundary for video input.
// No decoding or external media API is used at this stage.
class VideoSource {
public:
    bool Open(const std::string& sourcePath);
    void Update();
    const VideoFrame* GetFrame() const;
};
