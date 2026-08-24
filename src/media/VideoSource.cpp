#include "media/VideoSource.h"

bool VideoSource::Open(const std::string& sourcePath) {
    // TODO: Open a video stream through the selected media backend.
    (void)sourcePath;
    return false;
}

void VideoSource::Update() {
    // TODO: Decode or acquire the next video frame.
}

const VideoFrame* VideoSource::GetFrame() const {
    // TODO: Return the most recently acquired frame.
    return nullptr;
}
