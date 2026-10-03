#pragma once

#include "assets/GraphicsTypes.h"

#include <memory>
#include <unordered_map>

class Material;
class Mesh;


/*
    CPU-side ModelResource의 데이터를
    실제 Renderer가 사용할 GPU Resource로 변환/보관한다.

    GltfLoader
        ↓
    ModelResource
        ↓
    AssetManager
        ↓
    Mesh / Material
*/
class AssetManager final
{
public:
    AssetManager();

    /*
        ModelResource 내부의 MeshData / MaterialData를
        GPU-side Resource로 변환한다.

        MeshData::gpuMesh도 함께 채운다.
    */
    void UploadModel(ModelResource& model);

    std::shared_ptr<Mesh> GetMesh(ResourceID id) const;

    std::shared_ptr<Material> GetMaterial(ResourceID id) const;

    std::shared_ptr<Material> GetDefaultMaterial() const;

    void Clear();


private:
    std::unordered_map<ResourceID,std::shared_ptr<Mesh>> meshes_;

    std::unordered_map<ResourceID,std::shared_ptr<Material>> materials_;

    /*
        glTF Primitive에 Material이 지정되지 않은 경우 사용한다.
    */
    std::shared_ptr<Material> defaultMaterial_;
};