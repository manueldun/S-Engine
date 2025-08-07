#pragma once
#include "Triangle.h"
#include <X11/X.h>
#include <glm/glm.hpp>
#include <list>
#include <set>
#include <string>
#include <vector>

// #define QUICK_HULL_DEBUG
namespace ph {

class QhHalfEdge;
class QhVertex;
class QhFace {
public:
  QhFace() = default;
  QhFace *previous;
  QhFace *next;
  QhHalfEdge *edge;
  std::list<glm::vec3> conflictList;
  bool visited = false;

  bool isTowardsPoint(const glm::vec3 &point, const float &epsilon) const;
  float distanceToPoint(const glm::vec3 &point) const;
  glm::vec3 getFurtherVertex() const;
  void printVertices() const;
  std::list<QhHalfEdge *> getEdges() const;

private:
};
class QhHalfEdge {
public:
  QhHalfEdge() = default;
  QhVertex *tail;
  QhHalfEdge *previous;
  QhHalfEdge *twin;
  QhHalfEdge *next;
  QhFace *face;

private:
};
class QhVertex {
public:
  QhVertex() = default;
  glm::vec3 position;
  QhVertex *previous;
  QhVertex *next;
  QhHalfEdge *edge;
  static glm::vec3 getFarthestTowardsTriangle(const glm::vec3 &vertex1,
                                              const glm::vec3 &vertex2,
                                              const glm::vec3 &vertex3,
                                              std::vector<glm::vec3> &vertices,
                                              const float &epsilon);
  static glm::vec3 getFarthestFromLine(const glm::vec3 &qhVertex1,
                                       const glm::vec3 &qhVertex2,
                                       const std::list<glm::vec3> &vertices,
                                       const float &epsilon);
  static glm::vec3 getFarthestFromTriangle(const glm::vec3 &qhVertex1,
                                           const glm::vec3 &qhVertex2,
                                           const glm::vec3 &qhVertex3,
                                           const std::list<glm::vec3> &vertices,
                                           const float &epsilon);

  static float getDistancefromTriangle(const glm::vec3 &vertex1,
                                       const glm::vec3 &vertex2,
                                       const glm::vec3 &vertex3,
                                       const glm::vec3 &point,
                                       const float &epsilon);

private:
};
class Mesh {
public:
  Mesh(const std::vector<glm::vec3> &vertices,
       const std::vector<uint16_t> &indices);

  const std::vector<glm::vec3> vertices;
  const std::vector<uint16_t> indices;

private:
};
class TriangleList {
public:
  TriangleList() = default;
  TriangleList(const Triangle *const currentTriangle,
               const TriangleList *const neightbor1,
               const TriangleList *const neightbor2,
               const TriangleList *const neightbor3);
  TriangleList *neightbor1;
  TriangleList *neightbor2;
  TriangleList *neightbor3;
  TriangleList *currentTriangle;

private:
};
class QuickHull {
public:
  QuickHull(const std::vector<glm::vec3> &vertices, const std::string &name);

  void buildQuickHull();
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec3> getVertexBuffer();
  std::vector<size_t> getIndexBuffer();

  void printToObj(const std::string &name);

private:
  const float epsilon;
  const std::string m_name;
  void createMesh();
  std::vector<glm::vec3> m_hullVertices;
  std::vector<size_t> m_hullIndices;
  std::list<QhVertex> qhVertices;
  std::list<QhFace> qhHullFaces;
  std::list<QhHalfEdge> qhHalfHullEdges;
  std::list<QhFace *> qhFaceHullSet;
  std::list<QhHalfEdge *> qhHalfEdgeHullSet;
  float calculateEpsilon();
  void buildInitialHull();
  void deleteFace(QhFace *face);
  std::list<QhHalfEdge *> getHorizon(QhFace *face, const glm::vec3 &eye);
  void mergeHullToHorizon(std::list<QhHalfEdge *> horizon, const glm::vec3 &point);
  QhFace *getUnvisitedFace();
#ifdef QUICK_HULL_DEBUG

  void debugPrintdata();
#endif // QUICK_HULL_DEBUG
};
} // namespace ph
