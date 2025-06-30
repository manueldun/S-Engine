#pragma once
#include <array>
#include <cstdint>
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <span>
#include <string>
#include <vector>
#include "tiny_gltf.h"

std::vector<char> readFile(const std::string &filename);

constexpr bool isTowardsPlaneNormal(std::array<glm::vec3, 3> triangleplane,
                                    glm::vec3);

tinygltf::Model loadGltfFile(const std::string &path);

std::vector<std::span<glm::vec3>>
getVerticeData(const tinygltf::Model &model, const std::string &attributeName);

class IndexDataSpan {
public:

  template<typename T>
  std::span<T> getIndexSpan();

private:
  std::span<uint8_t> m_indexSpan;
  uint8_t m_bytesPerInt;
};
template<typename T>
std::vector<std::span<T>> getIndexData(const tinygltf::Model &model);

template<typename T>
constexpr glm::vec3 getCenterOfMass(const std::span<glm::vec3> &vertices,
                                    const std::span<T> &indices,
                                    const bool &verbose);
template<typename T>
constexpr glm::mat4 getInertiaTensor(const std::span<glm::vec3> &vertices,
                                     const std::span<T> &indices,
                                     const bool &verbose);
