#pragma once
#include "Triangle.h"
#include <X11/X.h>
#include <array>
#include <glm/glm.hpp>
#include <list>
#include <memory>
#include <string>
#include <vector>

// #define QUICK_HULL_DEBUG
namespace ph {

class HalfEdge;
class HalfEdgeVertex;
class HalfEdgeFace {
public:
  HalfEdgeFace() = default;
  HalfEdgeFace(const HalfEdgeFace &halfEdgeFace);

  void setEdge(HalfEdge *edge);
  bool isTowardsPoint(const glm::vec3 &point, const float &epsilon) const;
  float distanceToPoint(const glm::vec3 &point) const;
  glm::vec3 getFurtherVertex() const;
  void printVertices() const;
  std::list<HalfEdge *> getEdges() const;
  Triangle getTrianglePlane() const;
  bool hasConflictPoint() const;

  std::list<glm::vec3> m_conflictList;
  HalfEdge *m_edge = nullptr;
  bool m_visited = false;

private:
};
class HalfEdge {
public:
  HalfEdge() = default;
  HalfEdge(HalfEdgeFace *face, HalfEdgeVertex *tail);

  static void chainEdges(const std::list<HalfEdge *> edges);
  void setFace(HalfEdgeFace *face);
  void setNext(HalfEdge *next);
  void setPrevious(HalfEdge *previous);
  glm::vec3 getTail() const;
  glm::vec3 getHead() const;
  Triangle getTrianglePlane() const;
  HalfEdgeFace *m_face;
  HalfEdgeVertex *m_tail;
  HalfEdge *m_previous;
  HalfEdge *m_twin;
  HalfEdge *m_next;

private:
};
class HalfEdgeVertex {
public:
  HalfEdgeVertex() = default;
  HalfEdgeVertex(const HalfEdgeVertex &halfEdgeVertex);
  HalfEdgeVertex &operator=(const HalfEdgeVertex &halfEdgeVertex) = default;
  HalfEdgeVertex(const glm::vec3 &vertex);
  void setEdge(HalfEdge *edge);
  glm::vec3 position;
  static glm::vec3 getFarthestTowardsTriangle(const glm::vec3 &vertex1,
                                              const glm::vec3 &vertex2,
                                              const glm::vec3 &vertex3,
                                              std::vector<glm::vec3> &vertices,
                                              const float &epsilon);
  static glm::vec3 getFarthestFromLine(const glm::vec3 &vertex1,
                                       const glm::vec3 &vertex2,
                                       const std::list<glm::vec3> &vertices,
                                       const float &epsilon);
  static glm::vec3 getFarthestFromTriangle(const glm::vec3 &vertex1,
                                           const glm::vec3 &vertex2,
                                           const glm::vec3 &vertex3,
                                           const std::list<glm::vec3> &vertices,
                                           const float &epsilon);

  static float getDistancefromTriangle(const glm::vec3 &vertex1,
                                       const glm::vec3 &vertex2,
                                       const glm::vec3 &vertex3,
                                       const glm::vec3 &point,
                                       const float &epsilon);

  HalfEdge *m_edge;

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
class Collision {
public:
  ~Collision() = default;
  virtual glm::vec3 getPosition1() const = 0;
  virtual glm::vec3 getPosition2() const = 0;
  virtual glm::vec3 getNormal() const = 0;

private:
};
class PlanePointCollision : public Collision {
public:
  virtual ~PlanePointCollision() = default;
  PlanePointCollision(const glm::vec3 &point, const Triangle &trianglePlane);

  glm::vec3 getPosition1() const override;
  glm::vec3 getPosition2() const override;
  glm::vec3 getNormal() const override;
  const glm::vec3 m_point1;
  const Triangle m_trianglePlane2;

private:
};
class Witness {
public:
  Witness(const glm::vec3 &normal, const glm::vec3 &point);
  const glm::vec3 normal;
  const glm::vec3 point;
private:
};
class EdgeEdgeCollision : public Collision {
public:
  virtual ~EdgeEdgeCollision() = default;
  EdgeEdgeCollision(const std::array<glm::vec3, 2> &edge1,
                    const std::array<glm::vec3, 2> &edge2,
                    const glm::vec3 &normal);
  glm::vec3 getPosition1() const override;
  glm::vec3 getPosition2() const override;
  glm::vec3 getNormal() const override;

  const std::array<glm::vec3, 2> m_edge1;
  const std::array<glm::vec3, 2> m_edge2;
  const glm::vec3 m_normal;

private:
};
class QuickHull {
public:
  QuickHull(const std::vector<glm::vec3> &vertices, const std::string &name);
  QuickHull(const QuickHull &quickHull);
  QuickHull &operator=(const QuickHull &quickHull);
  ~QuickHull();

  const std::vector<glm::vec3> m_vertices;
  std::vector<glm::vec3> getVertexBuffer();
  std::vector<size_t> getIndexBuffer();

  void printToObj(const std::string &name);

  std::vector<std::shared_ptr<Collision>>
  getPlanePointCollisions(const glm::mat4 &thisTransform,
                          const QuickHull &ThatHull,
                          const glm::mat4 &thatTransform) const;

  std::vector<std::shared_ptr<Collision>>
  getEdgeEdgeCollisions(const glm::mat4 &thisTransform,
                        const QuickHull &ThatHull,
                        const glm::mat4 &thatTransform) const;
  std::shared_ptr<Witness> getWitness(const glm::mat4 &thisTransform,
                                        const QuickHull &thatHull,
                                        const glm::mat4 &thatTransform) const;

private:
  const float m_epsilon;
  const std::string m_name;
  std::vector<glm::vec3> m_hullVertices;
  std::vector<size_t> m_hullIndices;
  std::list<HalfEdgeVertex> m_halfEdgeVertices;
  std::list<HalfEdge *> m_hullHalfEdgesPtr;
  std::list<HalfEdgeFace *> m_hullHalEdgeFacesPtr;
  void buildQuickHull();
  void createMesh();
  float calculateEpsilon();
  void buildInitialHull();
  void deleteFace(HalfEdgeFace *face);
  std::list<HalfEdge *> getHorizon(HalfEdgeFace *face, const glm::vec3 &eye);
  void mergeHullToHorizon(std::list<HalfEdge *> horizon,
                          const glm::vec3 &point);
#ifdef QUICK_HULL_DEBUG

  void debugPrintdata();
#endif // QUICK_HULL_DEBUG
};
} // namespace ph
