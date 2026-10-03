#pragma once

#include "assets/GraphicsTypes.h"

#include <memory>
#include <unordered_map>

class Material;
class Mesh;
class Texture;

class AssetManager final
{
public:
    AssetManager();

    void UploadModel(ModelResource& model);

    std::shared_ptr<Mesh> GetMesh(ResourceID id) const;
    std::shared_ptr<Material> GetMaterial(ResourceID id) const;
    std::shared_ptr<Texture> GetTexture(ResourceID id) const;

    std::shared_ptr<Material> GetDefaultMaterial() const;

    void Clear();

private:
    std::unordered_map<ResourceID, std::shared_ptr<Mesh>> meshes_;
    std::unordered_map<ResourceID, std::shared_ptr<Material>> materials_;
    std::unordered_map<ResourceID, std::shared_ptr<Texture>> textures_;

    std::shared_ptr<Material> defaultMaterial_;
};