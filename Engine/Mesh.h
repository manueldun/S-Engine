#pragma once

#include "glm/glm.hpp"
#include <glm/gtc/quaternion.hpp>
#include "tiny_gltf.h"
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
namespace Engine {
class Image {
public:
  Image(const Image &image) = default;
  ~Image() = default;
  Image(const std::vector<uint8_t> &data, const int m_width, const int m_height,
        const int numOfComponents);

  const std::vector<uint8_t> data;
  const int width;
  const int height;
  const int numOfComponents;

private:
};
class PlainMaterial {
public:
  PlainMaterial(const PlainMaterial &PlainMaterial) = default;
  virtual ~PlainMaterial() = default;
  PlainMaterial(const std::array<double, 3> &color, const Image &image);

  const std::array<double, 3> color;
  const Image colorImage;

private:
};
class MeshData {
public:
  MeshData(const MeshData &meshData) = default;
  virtual ~MeshData() = default;
  MeshData(const std::vector<float> &position,
           const std::vector<uint16_t> &indices,
           const std::vector<float> &normal, const std::vector<float> &texCoord,
           const std::vector<float> &tangent);
  const std::vector<float> position;
  const std::vector<uint16_t> indices;
  const std::vector<float> normal;
  const std::vector<float> texCoord;
  const std::vector<float> tangent;

private:
};
class MeshNode {
public:
  MeshNode(const MeshNode &meshData) = default;
  ~MeshNode() = default;
  MeshNode(const MeshData &meshData, const std::string &name, const float &mass,
           const glm::vec3 &position, const glm::quat &orientation,
           glm::vec3 &scale, const std::shared_ptr<Image> &colorTexture,
           const std::vector<std::weak_ptr<MeshNode>> &children);
  std::vector<float> getVertices();

  const MeshData meshData;
  const std::string name;
  const float mass;
  const glm::vec3 position;
  const glm::quat orientation;
  const glm::vec3 scale;
  const std::shared_ptr<Image> colorTexture;
  const std::vector<std::weak_ptr<MeshNode>> children;

private:
};
class Scene {
public:
  Scene(const tinygltf::Model &model);
  std::vector<std::shared_ptr<MeshNode>> getMeshNodes() const;

private:
  std::vector<std::shared_ptr<MeshNode>> m_meshNodes;
};
} // namespace Engine
