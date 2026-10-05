#pragma once

#include "assets/GraphicsTypes.h"

#include <filesystem>

// GLB 파일을 읽어 CPU ModelResource로 바꾼다. GPU 객체는 여기서 만들지 않는다.
class GltfLoader final
{
public:
    // 지원하지 않거나 범위가 잘못된 입력은 runtime_error로 거부한다.
    static ModelResource LoadGLB(const std::filesystem::path& path);

private:
    GltfLoader() = delete;
};
