#pragma once
#include <cstdint>
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include "tiny_gltf.h"
#include <glm/glm.hpp>
#include <span>
#include <string>
#include <vector>

std::vector<char> readFile(const std::string &filename);

glm::mat3 star(const glm::vec3 &a);

bool isTowardsPlaneNormal(const std::span<const glm::vec3, 3> &trianglePlane,
                          const glm::vec3 &point);

tinygltf::Model loadGltfFile(const std::string &path);

const std::span<const glm::vec3>
getVertexData(const tinygltf::Model &model, const std::string &attributeName);

class IndexDataSpan {
public:
  IndexDataSpan(const std::span<const uint8_t> &indexSpan,
                const uint8_t &bytesperInt);
  const std::span<const uint8_t> getIndexSpan() const;
  const uint8_t getBytesPerInt() const;

private:
  const std::span<const uint8_t> m_indexSpan;
  const uint8_t m_bytesPerInt;
};

std::vector<IndexDataSpan> getIndexSpans(const tinygltf::Model &model);

const glm::vec3 getCenterOfMass(const std::span<const glm::vec3> &vertices,
                                const IndexDataSpan &dataSpan,
                                const bool &verbose);

const glm::mat3 getInertiaTensor(const std::span<const glm::vec3> &vertices,
                                 const IndexDataSpan &dataSpan,
                                 const bool &verbose);
template <typename T>
const glm::vec3 getCenterOfMass(const std::span<const glm::vec3> &vertices,
                                const std::span<T> &indices,
                                const bool &verbose);
template <typename T>
const glm::mat3 getInertiaTensor(const std::span<const glm::vec3> &vertices,
                                 const std::span<T> &indices,
                                 const bool &verbose);

const bool doCollide(const std::span<const glm::vec3> vertices1,
                     const std::span<const glm::vec3> vertices2);
const bool
doCollideTriangleMeshBased(const std::span<const glm::vec3> vertices1,
                           const std::span<const glm::vec3> vertices2);
template <typename T>
const bool doCollideIndexedTriangleMeshBased(
    const std::span<const glm::vec3> vertices1, const std::span<T> indices1,
    const std::span<const glm::vec3> vertices2, const std::span<T> indices2);
