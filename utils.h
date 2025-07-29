#pragma once
#include "imgui.h"
#include <cstdint>
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include "tiny_gltf.h"
#include <glm/glm.hpp>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

std::vector<char> readFile(const std::string &filenae);

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
// #define COLLISSION_DEBUG
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
class Tetrahedron;
enum EdgeId { FIRST, SECOND, THIRD };
class Triangle {
public:
  Triangle(const Triangle &triangle) = default;
  const Triangle &operator=(const Triangle &triangle);
  bool operator==(const Triangle &) const = default;
  Triangle(const glm::vec3 &vertex1, const glm::vec3 &vertex2,
           const glm::vec3 &vertex3);
  Triangle getOpositeHandednesTriangle() const;
  bool isTowards(const glm::vec3 &point, const float &epsilon = 0.0f) const;
  Tetrahedron getTetrahedron(const glm::vec3 &point) const;
  float getDistanceToPointFromPlane(const glm::vec3 &point) const;
  std::vector<glm::vec3>
  getAllPointTowards(const std::vector<glm::vec3> &points) const;
  std::optional<glm::vec3>
  getFurthestPointTowards(const std::vector<glm::vec3> &points) const;
  std::array<glm::vec3, 3> getVertices() const;
  glm::vec3 getNormal() const;
  std::array<glm::vec3, 2> getEdge(const EdgeId &edgeId);
  float getVerticesElipson() const;
  glm::vec3 getVectorWiseMax() const;

private:
  const glm::vec3 m_vertex1;
  const glm::vec3 m_vertex2;
  const glm::vec3 m_vertex3;
};
class TriangleGraph;
class Tetrahedron {
public:
  Tetrahedron() = default;
  Tetrahedron(const Tetrahedron &simplex3D) = default;
  const Tetrahedron &operator=(const Tetrahedron &simplex3D);
  Tetrahedron(const std::vector<Triangle> &triangles);
  Tetrahedron(const Triangle &triangle, const glm::vec3 &point);
  const std::vector<Triangle> getTriangles() const;
  TriangleGraph getTriangleGraph();

private:
  const std::vector<Triangle> m_triangles;
};

class TriangleGraph {
public:
  class Node {
  public:
    Node() = default;
    Node(const Node &node) = default;
    const Node &operator=(const Node &node);
    Node(const std::shared_ptr<Triangle> &triangle,
         const std::vector<std::shared_ptr<Node>> &adjacencyList);
    std::shared_ptr<Triangle> getTriangle() const;
    struct AdjacentNode {
      std::weak_ptr<Node> adjacencyNode;
      EdgeId id;
    };
    const std::vector<AdjacentNode> getAdjacencyList() const;

  private:
    std::weak_ptr<Triangle> m_triangle;
    std::vector<std::weak_ptr<Node>> m_adjacencyList;
  };
  TriangleGraph() = default;
  TriangleGraph(const TriangleGraph &edge) = default;
  const TriangleGraph &operator=(const TriangleGraph &triangleGraph);
  TriangleGraph(const std::vector<std::shared_ptr<TriangleGraph::Node>> &nodes);
  std::vector<std::shared_ptr<Triangle>> getTriangles() const;
  std::vector<std::shared_ptr<Node>> getNodes() const;
  TriangleGraph mergeHullWithPoint(const std::shared_ptr<Node> &triangle,
                                   const glm::vec3 &point);
  bool isConvex() const;

private:
  std::vector<std::shared_ptr<TriangleGraph::Node>> m_nodes;
  std::vector<std::shared_ptr<Triangle>> m_triangles;
  float m_elipson = 0.0f;
};

const std::vector<std::shared_ptr<Triangle>>
quickHullOld(const std::vector<const glm::vec3> &points);
