#pragma once
#include <array>
#include <cstdint>
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include "tiny_gltf.h"
#include <glm/glm.hpp>
#include <span>
#include <string>
#include <vector>

std::vector<char> readFile(const std::string &filename);

constexpr bool isTowardsPlaneNormal(std::array<glm::vec3, 3> triangleplane,
                                    glm::vec3);

tinygltf::Model loadGltfFile(const std::string &path);

std::vector<std::span<glm::vec3>>
getVerticeData(const tinygltf::Model &model, const std::string &attributeName);

class IndexDataSpan {
public:
  IndexDataSpan(const std::span<uint8_t> &indexSpan,
                const uint8_t &bytesperInt);
  std::span<uint8_t> getIndexSpan() const;
  constexpr uint8_t getBytesPerInt() const;
  enum MyEnum {

  };

private:
  std::span<uint8_t> m_indexSpan;
  uint8_t m_bytesPerInt;
};

std::vector<IndexDataSpan> getIndexSpans(const tinygltf::Model &model);

constexpr glm::vec3 getCenterOfMass(const std::span<glm::vec3> &vertices,
                                    const IndexDataSpan &dataSpan,
                                    const bool &verbose);
template <typename T>
constexpr glm::vec3 getCenterOfMass(const std::span<glm::vec3> &vertices,
                                    const std::span<T> &indices,
                                    const bool &verbose);
template <typename T>
constexpr glm::mat4 getInertiaTensor(const std::span<glm::vec3> &vertices,
                                     const std::span<T> &indices,
                                     const bool &verbose);
