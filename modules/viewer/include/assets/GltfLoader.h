#pragma once

#include "assets/GraphicsTypes.h"

#include <filesystem>


/**
 * @brief glTF / GLB 파일을 읽어서 ModelResource로 변환
 * 
 * @author 홍윤표
 * 
 * @details GLB -> TinyGLTF -> GltfLoader -> ModelResource
 * 
 * 책임 :
 *      - glTF Node 읽기
 *      - Mesh / Primitive 읽기
 *      - Vertex / Index 읽기
 *      - Material Factor 읽기
 *      - Node 계층 복원
 *      - Node Local Transform 읽기
 */


class GltfLoader final{
public:
    static ModelResource LoadGLB(const std::filesystem::path& path);
private:
    GltfLoader() = delete;    
};