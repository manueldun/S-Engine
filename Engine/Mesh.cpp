#include "Mesh.h"
#include "Physics.h"
#include "glm/ext/matrix_transform.hpp"
#include "glm/ext/vector_float3.hpp"
#include "glm/fwd.hpp"
#include "glm/gtc/type_ptr.hpp"
#include <memory>

namespace Engine {
Image::Image(const std::vector<uint8_t> &data, const int width,
             const int height, const int numbOfComponents)
    : data(data), width(width), height(height),
      numOfComponents(numbOfComponents) {}

PlainMaterial::PlainMaterial(const std::array<double, 3> &color,
                             const Image &image)
    : color(color), colorImage(image) {}

MeshData::MeshData(const std::vector<float> &position,
                   const std::vector<uint16_t> &indices,
                   const std::vector<float> &normal,
                   const std::vector<float> &texCoord,
                   const std::vector<float> &tangent)
    : position(position), indices(indices), normal(normal), texCoord(texCoord),
      tangent(tangent) {}

MeshNode::MeshNode(const MeshData &meshData, const std::string &name,
                   const float &mass, const glm::vec3 &position,
                   const glm::quat &orientation, glm::vec3 &scale,
                   const std::shared_ptr<Image> &colorTexture,
                   const std::vector<std::weak_ptr<MeshNode>> &children)
    : meshData(meshData), name(name), mass(mass), position(position),
      orientation(orientation), scale(scale), colorTexture(colorTexture),
      children(children) {}

Scene::Scene(const tinygltf::Model &model) {
  if (model.cameras.size() > 0) {
    tinygltf::Camera camera = model.cameras.at(0);
    if (camera.type.compare("perspective") == 0) {
      m_projectionCamera = ProjectionCamera(
          camera.perspective.aspectRatio, camera.perspective.yfov,
          camera.perspective.znear, camera.perspective.zfar);
    } else if (camera.type.compare("orthographic") == 0) {
      assert(false);
    }
  }
  std::vector<Image> images;
  images.reserve(model.images.size());
  for (const tinygltf::Image &image : model.images) {
    assert(image.component == 3 || image.component == 4);
    images.push_back(
        Image(image.image, image.width, image.height, image.component));
  }

  m_meshNodes.reserve(model.nodes.size());
  for (const tinygltf::Node &node : model.nodes) {
    if (node.mesh == -1) {
      if (node.camera != -1) {

        glm::vec3 scale = glm::vec3(1.0f, 1.0f, 1.0f);
        if (node.scale.size() != 0) {
          scale = glm::make_vec3(node.scale.data());
        }
        glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        if (node.rotation.size() != 0) {
          orientation = glm::make_quat(node.rotation.data());
        }
        glm::vec3 translation = glm::vec3(0.0f, 0.0f, 0.0f);
        if (node.translation.size() != 0) {
          translation = glm::make_vec3(node.translation.data());
        }

        glm::mat4 translationMat =
            glm::translate(glm::mat4(1.0f), -translation);
        glm::mat4 orientationMat = glm::transpose(glm::toMat4(orientation));
        glm::mat4 scaleMat = glm::scale(glm::mat4(1.0f), scale);
        m_viewMatrix = scaleMat * orientationMat * translationMat;
      }
      break;
    }
    tinygltf::Mesh mesh = model.meshes[node.mesh];

    std::string name = node.name;
    std::vector<float> positionBuffer;
    std::vector<float> normalBuffer;
    std::vector<float> texCoordBuffer;
    std::vector<float> tangentBuffer;
    std::vector<uint16_t> indexBuffer;
    assert(mesh.primitives.size() != 0);
    tinygltf::Primitive primitive = mesh.primitives.at(0);
    if (primitive.indices != -1) {
      tinygltf::Accessor indexAcessor = model.accessors[primitive.indices];
      assert(indexAcessor.componentType ==
             TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT);
      tinygltf::BufferView indexBufferView =
          model.bufferViews[indexAcessor.bufferView];
      tinygltf::Buffer buffer = model.buffers[indexBufferView.buffer];
      int offset = indexAcessor.byteOffset + indexBufferView.byteOffset;
      int byteLength = indexBufferView.byteLength;
      std::vector<uint8_t> indexByteBuffer(buffer.data.begin() + offset,
                                           buffer.data.begin() + offset +
                                               byteLength);
      indexBuffer.reserve(indexByteBuffer.size() / 2);
      for (size_t i = 0; i < indexByteBuffer.size(); i += 2) {
        uint16_t index = indexByteBuffer.at(i) | indexByteBuffer.at(i + 1) << 8;
        indexBuffer.push_back(index);
      }
      assert(indexBuffer.size() == byteLength / 2);
      struct AttributeData {
        const char *name;
        int componentType;
        int type;
        std::vector<float> &buffer;
      };
      std::vector<AttributeData> attributesData = {
          AttributeData{.name = "POSITION",
                        .componentType = TINYGLTF_COMPONENT_TYPE_FLOAT,
                        .type = TINYGLTF_TYPE_VEC3,
                        .buffer = positionBuffer},
          AttributeData{.name = "NORMAL",
                        .componentType = TINYGLTF_COMPONENT_TYPE_FLOAT,
                        .type = TINYGLTF_TYPE_VEC3,
                        .buffer = normalBuffer},
          AttributeData{.name = "TEXCOORD_0",
                        .componentType = TINYGLTF_COMPONENT_TYPE_FLOAT,
                        .type = TINYGLTF_TYPE_VEC2,
                        .buffer = texCoordBuffer},
          AttributeData{.name = "TANGENT",
                        .componentType = TINYGLTF_COMPONENT_TYPE_FLOAT,
                        .type = TINYGLTF_TYPE_VEC3,
                        .buffer = tangentBuffer}};
      for (const AttributeData &attributeData : attributesData) {
        if (primitive.attributes.count(attributeData.name) > 0) {
          assert(primitive.attributes.count(attributeData.name) == 1);
          tinygltf::Accessor accessor =
              model.accessors[primitive.attributes[attributeData.name]];
          tinygltf::BufferView bufferView =
              model.bufferViews[accessor.bufferView];
          tinygltf::Buffer buffer = model.buffers[bufferView.buffer];
          offset = accessor.byteOffset + bufferView.byteOffset;
          byteLength = bufferView.byteLength;
          assert(bufferView.byteStride == 0);
          std::vector<uint8_t> vertexBufferBuffer(buffer.data.begin() + offset,
                                                  buffer.data.begin() + offset +
                                                      byteLength);
          assert(accessor.componentType == attributeData.componentType);
          assert(accessor.type == attributeData.type);
          for (size_t i = 0; i < vertexBufferBuffer.size(); i += 4) {
            uint32_t vertexData = vertexBufferBuffer.at(i);
            vertexData |= vertexBufferBuffer.at(i + 1) << 8;
            vertexData |= vertexBufferBuffer.at(i + 2) << 16;
            vertexData |= vertexBufferBuffer.at(i + 3) << 24;
            attributeData.buffer.push_back(
                *reinterpret_cast<float *>(&vertexData));
          }
        }
        assert(positionBuffer.size() != 0);
        for (const AttributeData &attributeData : attributesData) {
          const int numberOfElements = 0;
          if (attributeData.buffer.size() != 0) {
            switch (attributeData.componentType) {
            case TINYGLTF_TYPE_VEC2: {
              assert(numberOfElements == attributesData.size() / 2);
              break;
            }
            case TINYGLTF_TYPE_VEC3: {
              assert(numberOfElements == attributesData.size() / 3);
              break;
            }
            }
          }
        }
      }
    }
    std::vector<std::weak_ptr<MeshNode>> meshNodeChildren;
    for (const int &nodeIndex : node.children) {
      meshNodeChildren.push_back(m_meshNodes.at(nodeIndex));
    }
    MeshData meshData(positionBuffer, indexBuffer, normalBuffer, texCoordBuffer,
                      tangentBuffer);
    tinygltf::Material material = model.materials[primitive.material];
    tinygltf::PbrMetallicRoughness pbrMaterial = material.pbrMetallicRoughness;
    std::vector<float> baseColorFactor(pbrMaterial.baseColorFactor.begin(),
                                       pbrMaterial.baseColorFactor.end());
    std::shared_ptr<Image> baseColorImage;
    if (images.size() > pbrMaterial.baseColorTexture.index) {
      baseColorImage = std::make_shared<Image>(
          images.at(pbrMaterial.baseColorTexture.index));
    } else {
      baseColorImage = std::make_shared<Image>(
          std::vector<uint8_t>({255, 0, 0, 0}), 1, 1, 4);
    }
    float mass = 0.0f;
    if (node.extras.Has("mass"))
      mass = node.extras.Get("mass").GetNumberAsDouble();
    glm::vec3 scale = glm::vec3(1.0f, 1.0f, 1.0f);
    if (node.scale.size() != 0) {
      scale = glm::make_vec3(node.scale.data());
    }
    glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    if (node.rotation.size() != 0) {
      orientation = glm::make_quat(node.rotation.data());
    }
    glm::vec3 translation = glm::vec3(0.0f, 0.0f, 0.0f);
    if (node.translation.size() != 0) {
      translation = glm::make_vec3(node.translation.data());
    }
    assert(node.matrix.size() == 0);
    m_meshNodes.push_back(std::make_shared<MeshNode>(
        meshData, node.name, mass, translation, orientation, scale,
        baseColorImage, meshNodeChildren));
  }
}
std::vector<std::shared_ptr<MeshNode>> Scene::getMeshNodes() const {
  return m_meshNodes;
}
glm::mat4 Scene::getCameraViewMatrix() const { return m_viewMatrix; }
ProjectionCamera &Scene::getProjectionCamera() { return m_projectionCamera; }
} // namespace Engine
