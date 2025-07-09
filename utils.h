#pragma once
#include <cstdint>
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include "tiny_gltf.h"
#include <glm/glm.hpp>
#include <iostream>
#include <span>
#include <string>
#include <vector>

std::vector<char> readFile(const std::string &filename);

glm::mat3 star(const glm::vec3 &a);

bool isTowardsPlaneNormal(const std::span<const glm::vec3> &trianglePlane,
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

template <typename T>
std::span<T> getindexSpans(const IndexDataSpan &indexSpan);

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
//#define COLLISSION_DEBUG
template <typename T1, typename T2>
inline bool doCollideIndexedTriangleMeshBased(
    const std::span<const glm::vec3> vertices1, const std::span<T1> indices1,
    const glm::mat4 transform1, const std::span<const glm::vec3> vertices2,
    const std::span<T2> indices2, const glm::mat4 transform2) {
  assert(indices1.size() % 3 == 0);
  assert(indices1.size() % 3 == 0);
  for (size_t i = 0; i < indices1.size(); i += 3) {
    const glm::vec3 vertex11 =
        transform1 * glm::vec4(vertices1[indices1[i]], 1.0f);
    const glm::vec3 vertex12 =
        transform1 * glm::vec4(vertices1[indices1[i + 1]], 1.0f);
    const glm::vec3 vertex13 =
        transform1 * glm::vec4(vertices1[indices1[i + 2]], 1.0f);
    const std::array<glm::vec3, 3> vertexPlane = {vertex11, vertex12, vertex13};
#ifdef COLLISSION_DEBUG
    std::cout << "Plane " << i / 3 << std::endl;
    std::cout << "vertex 11: " << vertex11.x << "," << vertex11.y << ","
              << vertex11.z << std::endl;
    std::cout << "vertex 12: " << vertex12.x << "," << vertex12.y << ","
              << vertex12.z << std::endl;
    std::cout << "vertex 13: " << vertex13.x << "," << vertex13.y << ","
              << vertex13.z << std::endl;
#endif // COLLISSION_DEBUG
    bool areTowards = true;
    for (auto &index : indices2) {
      const glm::vec3 vertex2 = transform2 * glm::vec4(vertices2[index], 1.0f);
#ifdef COLLISSION_DEBUG
      std::cout << "vertex 2: " << vertex2.x << "," << vertex2.y << ","
                << vertex2.z << std::endl;
#endif // COLLISSION_DEBUG
      if (!isTowardsPlaneNormal(std::span(vertexPlane), vertex2)) {
        areTowards = false;
        break;
      }
    }
    if (areTowards) {
      areTowards = false;
      for (size_t j = 0; j < indices1.size(); j++) {
        const glm::vec3 vertex1 =
            transform1 * glm::vec4(vertices1[indices1[j]], 1.0f);
#ifdef COLLISSION_DEBUG
        std::cout << "vertex 1: " << vertex1.x << "," << vertex1.y << ","
                  << vertex1.z << std::endl;
#endif // COLLISSION_DEBUG
        if (indices1[i] != indices1[j] && indices1[i + 1] != indices1[j] &&
            indices1[i + 2] != indices1[j] &&
            isTowardsPlaneNormal(std::span(vertexPlane), vertex1)) {
          areTowards = true;
          break;
        }
      }
      if (!areTowards) {
        return false;
      }
    }
  }
  return true;
}
