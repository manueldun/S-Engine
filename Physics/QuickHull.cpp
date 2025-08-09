#include "QuickHull.h"
#include "glm/common.hpp"
#include <iostream>
#include <map>
#include <sys/wait.h>

#ifdef QUICK_HULL_DEBUG
#include <iostream>
#endif // QUICK_HULL_DEBUG
namespace ph {
HalfEdgeFace::HalfEdgeFace(const HalfEdgeFace &halfEdgeFace) {
  m_conflictList = halfEdgeFace.m_conflictList;
  m_visited = halfEdgeFace.m_visited;
  std::list<HalfEdge *> newEdges;
  for (HalfEdge *edge : halfEdgeFace.getEdges()) {
    newEdges.push_back(new HalfEdge(this, new HalfEdgeVertex(*edge->m_tail)));
  }
  m_edge = newEdges.front();
  HalfEdge::chainEdges(newEdges);
}
void HalfEdgeFace::setEdge(HalfEdge *edge) { m_edge = edge; }
bool HalfEdgeFace::isTowardsPoint(const glm::vec3 &point,
                                  const float &epsilon) const {
  glm::vec3 point1 = m_edge->m_tail->position;
  glm::vec3 point2 = m_edge->m_next->m_tail->position;
  glm::vec3 point3 = m_edge->m_next->m_next->m_tail->position;
  Triangle triangle(point1, point2, point3);
  return triangle.isTowards(point, epsilon);
}
float HalfEdgeFace::distanceToPoint(const glm::vec3 &point) const {

  glm::vec3 point1 = m_edge->m_tail->position;
  glm::vec3 point2 = m_edge->m_next->m_tail->position;
  glm::vec3 point3 = m_edge->m_next->m_next->m_tail->position;
  Triangle triangle(point1, point2, point3);
  return triangle.getDistanceToPointFromPlane(point);
}

glm::vec3 HalfEdgeFace::getFurtherVertex() const {
  glm::vec3 point1 = m_edge->m_tail->position;
  glm::vec3 point2 = m_edge->m_next->m_tail->position;
  glm::vec3 point3 = m_edge->m_next->m_next->m_tail->position;
  Triangle triangle(point1, point2, point3);
  glm::vec3 further = m_conflictList.front();

  float maxDistance = triangle.getDistanceToPointFromPlane(further);
  for (const glm::vec3 &vertex : m_conflictList) {

    float distance = triangle.getDistanceToPointFromPlane(vertex);
    if (maxDistance < distance) {
      maxDistance = distance;
      further = vertex;
    }
  }
  return further;
}

void HalfEdgeFace::printVertices() const {
  HalfEdge *currentEdge = m_edge;
  HalfEdge *initialEdge = m_edge;
  do {
    std::cout << "v " << currentEdge->m_tail->position.x << " "
              << currentEdge->m_tail->position.y << " "
              << currentEdge->m_tail->position.z << std::endl;
    currentEdge = currentEdge->m_next;
  } while (currentEdge != initialEdge);
}

std::list<HalfEdge *> HalfEdgeFace::getEdges() const {
  HalfEdge *currentEdge = m_edge;
  HalfEdge *firstEdge = m_edge;
  std::list<HalfEdge *> edges;
  do {

    edges.push_back(currentEdge);
    currentEdge = currentEdge->m_next;
  } while (firstEdge != currentEdge);
  return edges;
}
Triangle HalfEdgeFace::getTrianglePlane() const {

  std::list<HalfEdge *> thisEdges = getEdges();
  std::vector<glm::vec3> thisVertices;
  for (HalfEdge *edge : thisEdges) {
    thisVertices.push_back(edge->m_tail->position);
  }
  return Triangle(thisVertices.at(0), thisVertices.at(1), thisVertices.at(2));
}
bool HalfEdgeFace::hasConflictPoint() const {
  return m_conflictList.size() != 0;
}
HalfEdge::HalfEdge(HalfEdgeFace *face, HalfEdgeVertex *tail)
    : m_face(face), m_tail(tail) {}
void HalfEdge::setFace(HalfEdgeFace *face) { m_face = face; }
void HalfEdge::setNext(HalfEdge *next) { m_next = next; }
void HalfEdge::setPrevious(HalfEdge *previous) { m_previous = previous; }
glm::vec3 HalfEdge::getTail() const { return m_tail->position; }
glm::vec3 HalfEdge::getHead() const { return m_next->m_tail->position; }
Triangle HalfEdge::getTrianglePlane() const {
  return m_face->getTrianglePlane();
}
void HalfEdge::chainEdges(const std::list<HalfEdge *> edges) {
  for (auto it = edges.begin(); it != edges.end(); ++it) {
    if (it != edges.begin()) {
      (*it)->m_previous = *(--it);
    }
    if (it != edges.end()) {
      (*it)->m_next = *(++it);
    }
  }
  edges.front()->m_previous = edges.back();
  edges.back()->m_next = edges.front();
}
glm::vec3 HalfEdgeVertex::getFarthestFromLine(
    const glm::vec3 &linePoint1, const glm::vec3 &linePoint2,
    const std::list<glm::vec3> &qhVertices, const float &epsilon) {

  glm::vec3 farthestPoint = qhVertices.front();
  glm::vec3 numeratorVec =
      glm::cross(farthestPoint - linePoint1, farthestPoint - linePoint2);
  glm::vec3 denominatorVec = linePoint2 - linePoint1;
  float maxDistance = glm::length(numeratorVec) / glm::length(denominatorVec);
  for (glm::vec3 point : qhVertices) {
    glm::vec3 numeratorVec = glm::cross(point - linePoint1, point - linePoint2);
    glm::vec3 denominatorVec = linePoint2 - linePoint1;
    float distance = glm::length(numeratorVec) / glm::length(denominatorVec);
    if (maxDistance < distance) {
      maxDistance = distance;
      farthestPoint = point;
    }
  }
  return farthestPoint;
}

HalfEdgeVertex::HalfEdgeVertex(const HalfEdgeVertex &halfEdgeVertex)
    : position(halfEdgeVertex.position) {
  m_edge = halfEdgeVertex.m_edge;
}
HalfEdgeVertex::HalfEdgeVertex(const glm::vec3 &vertex) { position = vertex; }
void HalfEdgeVertex::setEdge(HalfEdge *edge) { m_edge = edge; }
glm::vec3 HalfEdgeVertex::getFarthestTowardsTriangle(
    const glm::vec3 &vertex1, const glm::vec3 &vertex2,
    const glm::vec3 &vertex3, std::vector<glm::vec3> &vertices,
    const float &epsilon) {

  glm::vec3 furthest = vertices.front();

  float maxDistance = HalfEdgeVertex::getDistancefromTriangle(
      vertex1, vertex2, vertex3, furthest, epsilon);
  for (glm::vec3 &qhVertex : vertices) {

    float distanceFromPoint = HalfEdgeVertex::getDistancefromTriangle(
        vertex1, vertex2, vertex3, qhVertex, epsilon);
    if (maxDistance < distanceFromPoint) {
      maxDistance = distanceFromPoint;
      furthest = qhVertex;
    }
  }
  return furthest;
}
glm::vec3 HalfEdgeVertex::getFarthestFromTriangle(
    const glm::vec3 &vertex1, const glm::vec3 &vertex2,
    const glm::vec3 &vertex3, const std::list<glm::vec3> &vertices,
    const float &epsilon) {

  glm::vec3 furthest = vertices.front();

  float maxDistance = HalfEdgeVertex::getDistancefromTriangle(
      vertex1, vertex2, vertex3, furthest, epsilon);
  maxDistance = glm::abs(maxDistance);
  for (glm::vec3 qhVertex : vertices) {

    float distanceFromPoint = glm::abs(HalfEdgeVertex::getDistancefromTriangle(
        vertex1, vertex2, vertex3, qhVertex, epsilon));
    if (maxDistance < distanceFromPoint) {
      maxDistance = distanceFromPoint;
      furthest = qhVertex;
    }
  }
  return furthest;
}
float HalfEdgeVertex::getDistancefromTriangle(const glm::vec3 &vertex1,
                                              const glm::vec3 &vertex2,
                                              const glm::vec3 &vertex3,
                                              const glm::vec3 &point,
                                              const float &epsilon) {

  const glm::vec3 edge1 = vertex2 - vertex1;
  const glm::vec3 edge2 = vertex3 - vertex1;
  const glm::vec3 crossVector = glm::cross(edge1, edge2);
  const glm::vec3 normal = glm::normalize(crossVector);
  return glm::dot(normal, point - vertex1);
}
Mesh::Mesh(const std::vector<glm::vec3> &vertices,
           const std::vector<uint16_t> &indices)
    : vertices(vertices), indices(indices) {}

QuickHull::QuickHull(const std::vector<glm::vec3> &vertices,
                     const std::string &name)
    : m_vertices(vertices), m_epsilon(calculateEpsilon()), m_name(name) {
  buildQuickHull();
}

QuickHull::QuickHull(const QuickHull &quickHull)
    : m_vertices(quickHull.m_vertices), m_epsilon(quickHull.m_epsilon),
      m_name(quickHull.m_name) {

  m_halfEdgeVertices = quickHull.m_halfEdgeVertices;
  std::map<const HalfEdgeVertex *, HalfEdgeVertex *> vertexMap;
  auto thisIter = m_halfEdgeVertices.begin();
  auto thatIter = quickHull.m_halfEdgeVertices.begin();
  while (thisIter != m_halfEdgeVertices.end()) {
    HalfEdgeVertex *thisVertex = &(*thisIter);
    const HalfEdgeVertex *thatVertex = &(*thatIter);
    vertexMap[thatVertex] = thisVertex;
    ++thisIter;
    ++thatIter;
  }
  std::map<const HalfEdge *, HalfEdge *> edgeMap;
  std::map<const HalfEdgeFace *, HalfEdgeFace *> faceMap;
  for (HalfEdgeFace *face : quickHull.m_hullHalEdgeFacesPtr) {
    if (!faceMap.contains(face)) {
      HalfEdgeFace *newFace = new HalfEdgeFace();
      m_hullHalEdgeFacesPtr.push_back(newFace);
      faceMap[face] = newFace;
    }
    for (HalfEdge *edge : face->getEdges()) {
      HalfEdge *newEdge = new HalfEdge();
      m_hullHalfEdgesPtr.push_back(newEdge);
      edgeMap[edge] = newEdge;
    }
  }
  for (HalfEdgeFace *face : quickHull.m_hullHalEdgeFacesPtr) {
    faceMap[face]->m_edge = edgeMap[face->m_edge];
  }
  for (const HalfEdgeVertex &vertex : quickHull.m_halfEdgeVertices) {

    vertexMap[&vertex]->m_edge = edgeMap[vertex.m_edge];
  }
  for (HalfEdge *edge : quickHull.m_hullHalfEdgesPtr) {
    edgeMap[edge]->m_tail = vertexMap[edge->m_tail];
    edgeMap[edge]->m_next = edgeMap[edge->m_next];
    edgeMap[edge]->m_previous = edgeMap[edge->m_previous];
    edgeMap[edge]->m_twin = edgeMap[edge->m_twin];
  }
}
QuickHull::~QuickHull() {
  for (HalfEdge *edge : m_hullHalfEdgesPtr) {
    delete edge;
  }
  for (HalfEdgeFace *face : m_hullHalEdgeFacesPtr) {
    delete face;
  }
}
QuickHull &QuickHull::operator=(const QuickHull &quickHull) {
  for (HalfEdgeFace *face : quickHull.m_hullHalEdgeFacesPtr) {
    m_hullHalEdgeFacesPtr.push_back(new HalfEdgeFace(*face));
  }
  for (HalfEdgeFace *face : m_hullHalEdgeFacesPtr) {
    for (HalfEdge *edge : face->getEdges()) {
      m_hullHalfEdgesPtr.push_back(edge);
    }
  }
  m_halfEdgeVertices = quickHull.m_halfEdgeVertices;
  m_hullVertices = quickHull.m_hullVertices;
  m_hullIndices = quickHull.m_hullIndices;
  return *this;
}

void QuickHull::buildQuickHull() {

  buildInitialHull();
  int iteration = 1;
  printToObj(m_name + std::to_string(0));
  HalfEdgeFace *nextFace = nullptr;
  do {
    nextFace = nullptr;
    for (HalfEdgeFace *face : m_hullHalEdgeFacesPtr) {
      if (face->hasConflictPoint()) {
        nextFace = face;
        break;
      }
    }
    if (nextFace != nullptr) {
      glm::vec3 furthestVertex = nextFace->getFurtherVertex();
      std::cout << "#New Point: " << furthestVertex.x << "," << furthestVertex.y
                << "," << furthestVertex.z << "," << std::endl;
      std::list<HalfEdge *> horizon = getHorizon(nextFace, furthestVertex);
      assert(horizon.size() >= 3);
      std::cout << "#horizon:" << std::endl;
      for (HalfEdge *edge : horizon) {
        std::cout << "#(" << edge->getTail().x << "," << edge->getTail().y
                  << "," << edge->getTail().z << ")" << std::endl;
      }
      mergeHullToHorizon(horizon, furthestVertex);
      printToObj(m_name + std::to_string(iteration));
      iteration++;
    }
  } while (nextFace != nullptr);
  createMesh();
}

std::vector<glm::vec3> QuickHull::getVertexBuffer() { return m_hullVertices; }
std::vector<size_t> QuickHull::getIndexBuffer() { return m_hullIndices; }

std::vector<PlanePointCollision>
QuickHull::getPlanePointCollisions(const glm::mat4 &thisTransform,
                                   const QuickHull &thatHull,
                                   const glm::mat4 &thatTransform) const {
  std::vector<PlanePointCollision> collisions;

  for (HalfEdgeFace *thisFace : m_hullHalEdgeFacesPtr) {

    Triangle thisTriangle =
        thisFace->getTrianglePlane().transform(thisTransform);

    bool isInside = true;
    glm::vec3 thatVertex;
    for (const HalfEdgeVertex &thatQhVertex : thatHull.m_halfEdgeVertices) {

      thatVertex = thatTransform * glm::vec4(thatQhVertex.position, 1.0f);

      if (thisTriangle.isTowards(thatVertex, 0.0f)) {

        isInside = false;
        break;
      }
    }
    if (isInside) {
      collisions.push_back(PlanePointCollision{.point = thatVertex,
                                               .trianglePlane = thisTriangle});
    }
  }
  return collisions;
}
std::vector<EdgeEdgeCollision>
QuickHull::getEdgeEdgeCollisions(const glm::mat4 &thisTransform,
                                 const QuickHull &thatHull,
                                 const glm::mat4 &thatTransform) const {
  std::vector<EdgeEdgeCollision> collisions;
  for (HalfEdge *thatEdge : thatHull.m_hullHalfEdgesPtr) {
    std::array<glm::vec3, 2> thatEdgePoints = {thatEdge->getTail(),
                                               thatEdge->getHead()};
    bool isInside = true;
    std::array<glm::vec3, 2> thisEdgePoints;
    for (HalfEdge *thisEdge : m_hullHalfEdgesPtr) {
      thisEdgePoints = {thisEdge->getTail(), thisEdge->getHead()};
      glm::vec3 normal = thisEdge->m_face->getTrianglePlane()
                             .transform(thisTransform)
                             .getNormal();
      glm::vec3 normalTwin = thisEdge->m_twin->m_face->getTrianglePlane()
                                 .transform(thisTransform)
                                 .getNormal();
      glm::vec3 thisEdgeNormal = (normal + normalTwin) / 2.0f;
      float dotProduct = glm::dot(
          thisEdgeNormal, thatEdgePoints.at(0) - thisEdge->m_tail->position);
      if (dotProduct > 0.0f) {
        isInside = false;
        break;
      }
    }
    if (isInside) {
      collisions.push_back(
          EdgeEdgeCollision{.edge1 = thisEdgePoints, .edge2 = thatEdgePoints});
    }
  }
  return collisions;
}
void QuickHull::createMesh() {
  std::map<const HalfEdgeVertex *const, size_t> indicesMap;
  size_t index = 0;
  for (const HalfEdgeFace *face : m_hullHalEdgeFacesPtr) {
    std::list<HalfEdge *> edges = face->getEdges();
    assert(edges.size() == 3);
    for (const HalfEdge *const edge : edges) {
      if (!indicesMap.contains(edge->m_tail)) {
        indicesMap[edge->m_tail] = index;
        m_hullIndices.push_back(index);
        index++;
      } else {
        m_hullIndices.push_back(indicesMap.at(edge->m_tail));
      }
    }
  }
  for (const HalfEdgeVertex &vertex : m_halfEdgeVertices) {
    m_hullVertices.push_back(vertex.position);
  }
}
void QuickHull::mergeHullToHorizon(std::list<HalfEdge *> horizon,
                                   const glm::vec3 &point) {
  HalfEdge *previousHalfEdge = nullptr;
  HalfEdge *firstHalfEdge = nullptr;
  HalfEdge *firstHalfEdgeTwin = nullptr;
  std::list<HalfEdgeFace *> newFaces;

  m_halfEdgeVertices.push_back(HalfEdgeVertex(point));
  std::vector<HalfEdgeFace *> debugNewFaces;
  for (HalfEdge *horizonEdge : horizon) {
    HalfEdgeFace *currentFace = new HalfEdgeFace();
    m_hullHalEdgeFacesPtr.push_back(currentFace);
    debugNewFaces.push_back(currentFace);
    newFaces.push_back(currentFace);
    HalfEdge *edge1 = new HalfEdge(currentFace, horizonEdge->m_next->m_tail);
    currentFace->setEdge(edge1);
    HalfEdge *edge2 = new HalfEdge(currentFace, horizonEdge->m_tail);
    HalfEdge *edge3 = new HalfEdge(currentFace, &m_halfEdgeVertices.back());
    m_hullHalfEdgesPtr.push_back(edge1);
    m_hullHalfEdgesPtr.push_back(edge2);
    m_hullHalfEdgesPtr.push_back(edge3);
    horizonEdge->m_twin = edge1;
    edge1->m_twin = horizonEdge;
    currentFace->m_edge = edge1;
    edge1->setNext(edge2);
    edge2->setNext(edge3);
    edge3->setNext(edge1);
    edge1->setPrevious(edge3);
    edge2->setPrevious(edge1);
    edge3->setPrevious(edge2);
    if (previousHalfEdge != nullptr) {
      edge3->m_twin = previousHalfEdge;
      previousHalfEdge->m_twin = edge3;
    }
    if (firstHalfEdge == nullptr) {
      firstHalfEdge = edge3;
    }
    previousHalfEdge = edge2;
    firstHalfEdgeTwin = edge2;
  }
  firstHalfEdge->m_twin = firstHalfEdgeTwin;
  firstHalfEdgeTwin->m_twin = firstHalfEdge;
  for (HalfEdgeFace *face : debugNewFaces) {
    std::cout << "#New Face:" << std::endl;
    std::cout << "#(" << face->m_edge->m_tail->position.x << ","
              << face->m_edge->m_tail->position.y << ","
              << face->m_edge->m_tail->position.z << ")" << std::endl;
    std::cout << "#(" << face->m_edge->m_next->m_tail->position.x << ","
              << face->m_edge->m_next->m_tail->position.y << ","
              << face->m_edge->m_next->m_tail->position.z << ")" << std::endl;
    std::cout << "#(" << face->m_edge->m_next->m_next->m_tail->position.x << ","
              << face->m_edge->m_next->m_next->m_tail->position.y << ","
              << face->m_edge->m_next->m_next->m_tail->position.z << ")"
              << std::endl;

    std::cout << "\t#Twin 1:" << std::endl;
    for (HalfEdge *twinEdge : face->m_edge->m_twin->m_face->getEdges()) {
      std::cout << "\t#(" << twinEdge->m_tail->position.x << ","
                << twinEdge->m_tail->position.y << ","
                << twinEdge->m_tail->position.z << ")" << std::endl;
    }
    std::cout << "\t#Twin 2:" << std::endl;
    for (HalfEdge *twinEdge :
         face->m_edge->m_next->m_twin->m_face->getEdges()) {
      std::cout << "\t#(" << twinEdge->m_tail->position.x << ","
                << twinEdge->m_tail->position.y << ","
                << twinEdge->m_tail->position.z << ")" << std::endl;
    }
    std::cout << "\t#Twin 3:" << std::endl;
    for (HalfEdge *twinEdge :
         face->m_edge->m_next->m_next->m_twin->m_face->getEdges()) {
      std::cout << "\t#(" << twinEdge->m_tail->position.x << ","
                << twinEdge->m_tail->position.y << ","
                << twinEdge->m_tail->position.z << ")" << std::endl;
    }
  }

  std::vector<HalfEdge *> edgesToErase;
  for (auto it = m_hullHalEdgeFacesPtr.begin();
       it != m_hullHalEdgeFacesPtr.end();) {
    HalfEdgeFace *face = *it;
    if (face->m_visited) {
      HalfEdge *currentEdge = face->m_edge;
      HalfEdge *startEdge = face->m_edge;
      do {
        edgesToErase.push_back(currentEdge);
        currentEdge = currentEdge->m_next;

      } while (currentEdge != startEdge);
      it = m_hullHalEdgeFacesPtr.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = m_hullHalfEdgesPtr.begin(); it != m_hullHalfEdgesPtr.end();) {
    auto halfEdgeIt = it;
    ++it;
    for (HalfEdge *edgeToErase : edgesToErase) {
      if (*halfEdgeIt == edgeToErase) {
        m_hullHalfEdgesPtr.erase(halfEdgeIt);
      }
    }
  }
  for (HalfEdgeFace *face : newFaces) {
    for (const glm::vec3 &vertex : m_vertices) {
      float distance = face->distanceToPoint(vertex);
      if (distance > m_epsilon) {
        face->m_conflictList.push_back(vertex);
      }
    }
  }

#ifdef QUICK_HULL_DEBUG
  debugPrintdata();

#endif // QUICK_HULL_DEBUG
}

void QuickHull::printToObj(const std::string &name) {
  std::cout << "o " << name << std::endl;
  std::vector<std::vector<int>> faces;
  std::map<HalfEdgeVertex *, int> indicesMap;
  std::vector<glm::vec3> vertices;
  static int vertexIndex = 1;
  for (const HalfEdgeFace *face : m_hullHalEdgeFacesPtr) {

    HalfEdgeVertex *vertex = face->m_edge->m_tail;
    std::vector<int> faceIndices;
    if (indicesMap.contains(vertex)) {
      faceIndices.push_back(indicesMap.at(vertex));
    } else {
      faceIndices.push_back(vertexIndex);
      indicesMap[vertex] = vertexIndex;
      vertexIndex++;
      vertices.push_back(vertex->position);
    }
    vertex = face->m_edge->m_next->m_tail;
    if (indicesMap.contains(vertex)) {
      faceIndices.push_back(indicesMap.at(vertex));
    } else {
      faceIndices.push_back(vertexIndex);
      indicesMap[vertex] = vertexIndex;
      vertexIndex++;
      vertices.push_back(vertex->position);
    }
    vertex = face->m_edge->m_next->m_next->m_tail;
    if (indicesMap.contains(vertex)) {
      faceIndices.push_back(indicesMap.at(vertex));
    } else {
      faceIndices.push_back(vertexIndex);
      indicesMap[vertex] = vertexIndex;
      vertexIndex++;
      vertices.push_back(vertex->position);
    }
    faces.push_back(faceIndices);
  }
  for (const glm::vec3 v : vertices) {
    std::cout << "v " << v.x << " " << v.y << " " << v.z << std::endl;
  }
  for (std::vector<int> &f : faces) {
    std::cout << "f ";
    for (int &index : f) {
      std::cout << index << " ";
    }
    std::cout << std::endl;
  }
}
#ifdef QUICK_HULL_DEBUG

void QuickHull::debugPrintdata() {
  int faceIndex = 0;
  std::cout << "=====================================" << std::endl;
  for (const QhFace *face : qhFaceHullSet) {
    std::cout << "Face " << faceIndex + 1 << ":" << std::endl;
    faceIndex++;
    glm::vec3 vertex1 = face->edge->tail->position;
    std::cout << "Edge 1 tail:" << std::endl;
    std::cout << "\tx: " << vertex1.x << " y: " << vertex1.y
              << " z: " << vertex1.z << std::endl;
    std::cout << "Edge 2 tail:" << std::endl;
    vertex1 = face->edge->next->tail->position;
    std::cout << "\tx: " << vertex1.x << " y: " << vertex1.y
              << " z: " << vertex1.z << std::endl;
    std::cout << "Edge 3 tail:" << std::endl;
    vertex1 = face->edge->next->next->tail->position;
    std::cout << "\tx: " << vertex1.x << " y: " << vertex1.y
              << " z: " << vertex1.z << std::endl;
  }
  int index = 0;
  for (const QhHalfEdge *edge : qhHalfEdgeHullSet) {
    std::cout << "Index " << index << std::endl;
    index++;
    if (edge->twin == nullptr) {
      std::cout << "nullptr found on a edge twin" << std::endl;
    }
    if (edge->tail == nullptr) {
      std::cout << "nullptr found on a edge tail" << std::endl;
    }
    if (edge->face == nullptr) {
      std::cout << "nullptr found on a edge face" << std::endl;
    }
  }
}

#endif // QUICK_HULL_DEBUG

void QuickHull::buildInitialHull() {
#ifdef QUICK_HULL_DEBUG
  std::cout << "Start of initial Hull" << std::endl;
#endif // QUICK_HULL_DEBUG

  glm::vec3 farthestXPositive = m_vertices.front();
  for (const glm::vec3 &vertex : m_vertices) {
    if (vertex.x > farthestXPositive.x) {
      farthestXPositive = vertex;
    }
  }
  glm::vec3 farthestXNegative = m_vertices.front();
  for (const glm::vec3 &vertex : m_vertices) {
    if (vertex.x < farthestXNegative.x) {
      farthestXNegative = vertex;
    }
  }
  glm::vec3 farthestYPositive = m_vertices.front();
  for (const glm::vec3 &vertex : m_vertices) {
    if (vertex.y > farthestYPositive.y) {
      farthestYPositive = vertex;
    }
  }
  glm::vec3 farthestYNegative = m_vertices.front();
  for (const glm::vec3 &vertex : m_vertices) {
    if (vertex.y < farthestYNegative.y) {
      farthestYNegative = vertex;
    }
  }
  glm::vec3 farthestZPositive = m_vertices.front();
  for (const glm::vec3 &vertex : m_vertices) {
    if (vertex.z > farthestZPositive.z) {
      farthestZPositive = vertex;
    }
  }
  glm::vec3 farthestZNegative = m_vertices.front();
  for (const glm::vec3 &vertex : m_vertices) {
    if (vertex.z < farthestYNegative.z) {
      farthestZNegative = vertex;
    }
  }
  glm::vec3 firstVertex;
  glm::vec3 secondVertex;
  glm::vec3 thirdVertex;
  glm::vec3 fourthVertex;
  float maxX = farthestXPositive.x - farthestXNegative.x;
  float maxY = farthestYPositive.y - farthestYNegative.y;
  float maxZ = farthestZPositive.z - farthestZNegative.z;
  if (maxX > maxY && maxX > maxZ) {
    firstVertex = farthestXPositive;
    secondVertex = farthestXNegative;

    std::list<glm::vec3> qhVert = {farthestYPositive, farthestYNegative,
                                   farthestZPositive, farthestZNegative};
    thirdVertex = HalfEdgeVertex::getFarthestFromLine(firstVertex, secondVertex,
                                                      qhVert, m_epsilon);
    fourthVertex = HalfEdgeVertex::getFarthestFromTriangle(
        firstVertex, secondVertex, thirdVertex, qhVert, m_epsilon);

  } else if (maxY > maxX && maxY > maxZ) {
    firstVertex = farthestYPositive;
    secondVertex = farthestYNegative;
    std::list<glm::vec3> qhVert = {farthestXPositive, farthestXNegative,
                                   farthestZPositive, farthestZNegative};
    thirdVertex = HalfEdgeVertex::getFarthestFromLine(firstVertex, secondVertex,
                                                      qhVert, m_epsilon);
    fourthVertex = HalfEdgeVertex::getFarthestFromTriangle(
        firstVertex, secondVertex, thirdVertex, qhVert, m_epsilon);
  } else {
    firstVertex = farthestZPositive;
    secondVertex = farthestZNegative;
    std::list<glm::vec3> qhVert = {farthestXPositive, farthestXNegative,
                                   farthestYPositive, farthestYNegative};
    thirdVertex = HalfEdgeVertex::getFarthestFromLine(firstVertex, secondVertex,
                                                      qhVert, m_epsilon);
    fourthVertex = HalfEdgeVertex::getFarthestFromTriangle(
        firstVertex, secondVertex, thirdVertex, qhVert, m_epsilon);
  }

  m_halfEdgeVertices.push_back(HalfEdgeVertex(firstVertex));
  HalfEdgeVertex *firstQhVertex = &m_halfEdgeVertices.back();
  m_halfEdgeVertices.push_back(HalfEdgeVertex(secondVertex));
  HalfEdgeVertex *secondQhVertex = &m_halfEdgeVertices.back();
  m_halfEdgeVertices.push_back(HalfEdgeVertex(thirdVertex));
  HalfEdgeVertex *thirdQhVertex = &m_halfEdgeVertices.back();
  m_halfEdgeVertices.push_back(HalfEdgeVertex(fourthVertex));
  HalfEdgeVertex *fourthQhVertex = &m_halfEdgeVertices.back();
#ifdef QUICK_HULL_DEBUG
  std::cout << "Initial hull vertices:" << std::endl;
  std::cout << "Vertex1:" << std::endl;
  std::cout << "\tx: (" << firstVertex.x << ",";
  std::cout << "y: " << firstVertex.y << ",";
  std::cout << "z: " << firstVertex.z << ")\n";
  std::cout << "Vertex2:" << std::endl;
  std::cout << "\tx: (" << secondVertex.x << ",";
  std::cout << "y: " << secondVertex.y << ",";
  std::cout << "z: " << secondVertex.z << ")\n";
  std::cout << "Vertex3:" << std::endl;
  std::cout << "\tx: (" << thirdVertex.x << ",";
  std::cout << "y: " << thirdVertex.y << ",";
  std::cout << "z: " << thirdVertex.z << ")\n";
  std::cout << "Vertex4:" << std::endl;
  std::cout << "\tx: (" << fourthVertex.x << ",";
  std::cout << "y: " << fourthVertex.y << ",";
  std::cout << "z: " << fourthVertex.z << ")\n";
#endif // QUICK_HULL_DEBUG

  HalfEdgeFace *face1 = new HalfEdgeFace();
  HalfEdgeFace *face2 = new HalfEdgeFace();
  HalfEdgeFace *face3 = new HalfEdgeFace();
  HalfEdgeFace *face4 = new HalfEdgeFace();

  m_hullHalEdgeFacesPtr.push_back(face1);
  m_hullHalEdgeFacesPtr.push_back(face2);
  m_hullHalEdgeFacesPtr.push_back(face3);
  m_hullHalEdgeFacesPtr.push_back(face4);

  HalfEdgeVertex *face1FirstVertex = firstQhVertex;
  HalfEdgeVertex *face1SecondVertex = secondQhVertex;
  HalfEdgeVertex *face1ThirdVertex = thirdQhVertex;
  HalfEdgeVertex *face1FourthVertex = fourthQhVertex;
  Triangle firstTriangle(face1FirstVertex->position,
                         face1SecondVertex->position,
                         face1ThirdVertex->position);
  if (firstTriangle.isTowards(face1FourthVertex->position)) {
    HalfEdgeVertex *tempVert = face1FirstVertex;
    face1FirstVertex = face1SecondVertex;
    face1SecondVertex = tempVert;
  }
  // face1
  HalfEdge *face1Edge1 = new HalfEdge(face1, face1FirstVertex);
  m_hullHalfEdgesPtr.push_back(face1Edge1);
  face1->m_edge = face1Edge1;

  HalfEdge *face1Edge2 = new HalfEdge(face1, face1SecondVertex);
  m_hullHalfEdgesPtr.push_back(face1Edge2);

  HalfEdge *face1Edge3 = new HalfEdge(face1, face1ThirdVertex);
  m_hullHalfEdgesPtr.push_back(face1Edge3);

  face1Edge1->m_next = face1Edge2;
  face1Edge2->m_next = face1Edge3;
  face1Edge3->m_next = face1Edge1;

  face1Edge1->m_previous = face1Edge3;
  face1Edge2->m_previous = face1Edge1;
  face1Edge3->m_previous = face1Edge2;

  HalfEdgeVertex *face2FirstVertex = firstQhVertex;
  HalfEdgeVertex *face2SecondVertex = secondQhVertex;
  HalfEdgeVertex *face2ThirdVertex = thirdQhVertex;
  HalfEdgeVertex *face2FourthVertex = fourthQhVertex;
  Triangle secondTriangle(face2SecondVertex->position,
                          face2ThirdVertex->position,
                          face2FourthVertex->position);
  if (secondTriangle.isTowards(face2FirstVertex->position)) {
    HalfEdgeVertex *tempVert = face2SecondVertex;
    face2SecondVertex = face2ThirdVertex;
    face2ThirdVertex = tempVert;
  }
  // face2
  HalfEdge *face2Edge1 = new HalfEdge(face2, face2SecondVertex);
  m_hullHalfEdgesPtr.push_back(face2Edge1);
  face2->m_edge = face2Edge1;

  HalfEdge *face2Edge2 = new HalfEdge(face2, face2ThirdVertex);
  m_hullHalfEdgesPtr.push_back(face2Edge2);

  HalfEdge *face2Edge3 = new HalfEdge(face2, face2FourthVertex);
  m_hullHalfEdgesPtr.push_back(face2Edge3);

  face2Edge1->setNext(face2Edge2);
  face2Edge2->setNext(face2Edge3);
  face2Edge3->setNext(face2Edge1);

  face2Edge1->setPrevious(face2Edge3);
  face2Edge2->setPrevious(face2Edge1);
  face2Edge3->setPrevious(face2Edge2);

  HalfEdgeVertex *face3FirstVertex = firstQhVertex;
  HalfEdgeVertex *face3SecondVertex = secondQhVertex;
  HalfEdgeVertex *face3ThirdVertex = thirdQhVertex;
  HalfEdgeVertex *face3FourthVertex = fourthQhVertex;
  Triangle thirdTriangle(face3FirstVertex->position,
                         face3SecondVertex->position,
                         face3FourthVertex->position);
  if (thirdTriangle.isTowards(face3ThirdVertex->position)) {
    HalfEdgeVertex *tempVert = face3FirstVertex;
    face3FirstVertex = face3SecondVertex;
    face3SecondVertex = tempVert;
  }
  // face3
  HalfEdge *face3Edge1 = new HalfEdge(face3, face3FirstVertex);

  m_hullHalfEdgesPtr.push_back(face3Edge1);
  face3->m_edge = face3Edge1;

  HalfEdge *face3Edge2 = new HalfEdge(face3, face3SecondVertex);
  m_hullHalfEdgesPtr.push_back(face3Edge2);

  HalfEdge *face3Edge3 = new HalfEdge(face3, face3FourthVertex);
  m_hullHalfEdgesPtr.push_back(face3Edge3);

  face3Edge1->setNext(face3Edge2);
  face3Edge2->setNext(face3Edge3);
  face3Edge3->setNext(face3Edge1);

  face3Edge1->setPrevious(face3Edge3);
  face3Edge2->setPrevious(face3Edge1);
  face3Edge3->setPrevious(face3Edge2);

  HalfEdgeVertex *face4FirstVertex = firstQhVertex;
  HalfEdgeVertex *face4SecondVertex = secondQhVertex;
  HalfEdgeVertex *face4ThirdVertex = thirdQhVertex;
  HalfEdgeVertex *face4FourthVertex = fourthQhVertex;
  Triangle fourthTriangle(face4FirstVertex->position,
                          face4ThirdVertex->position,
                          face4FourthVertex->position);
  if (fourthTriangle.isTowards(face4SecondVertex->position)) {
    HalfEdgeVertex *tempVert = face4FirstVertex;
    face4FirstVertex = face4ThirdVertex;
    face4ThirdVertex = tempVert;
  }
  // face4
  HalfEdge *face4Edge1 = new HalfEdge(face4, face4FirstVertex);
  m_hullHalfEdgesPtr.push_back(face4Edge1);
  face4->m_edge = face4Edge1;

  HalfEdge *face4Edge2 = new HalfEdge(face4, face4ThirdVertex);
  m_hullHalfEdgesPtr.push_back(face4Edge2);

  HalfEdge *face4Edge3 = new HalfEdge(face4, face4FourthVertex);
  m_hullHalfEdgesPtr.push_back(face4Edge3);

  face4Edge1->setNext(face4Edge2);
  face4Edge2->setNext(face4Edge3);
  face4Edge3->setNext(face4Edge1);

  face4Edge1->setPrevious(face4Edge3);
  face4Edge2->setPrevious(face4Edge1);
  face4Edge3->setPrevious(face4Edge2);

  for (HalfEdge *edge1 : m_hullHalfEdgesPtr) {

    for (HalfEdge *edge2 : m_hullHalfEdgesPtr) {
      if (edge1->m_tail == edge2->m_next->m_tail &&
          edge1->m_next->m_tail == edge2->m_tail) {
        edge1->m_twin = edge2;
        edge2->m_twin = edge1;
      }
    }
  }

#ifdef QUICK_HULL_DEBUG
  debugPrintdata();
#endif // QUICK_HULL_DEBUG

  for (const glm::vec3 vertex : m_vertices) {
    float distanceToFace1 = face1->distanceToPoint(vertex);
    float distanceToFace2 = face2->distanceToPoint(vertex);
    float distanceToFace3 = face3->distanceToPoint(vertex);
    float distanceToFace4 = face4->distanceToPoint(vertex);
    if (distanceToFace1 > m_epsilon) {
      face1->m_conflictList.push_back(vertex);
    }
    if (distanceToFace2 > m_epsilon) {
      face2->m_conflictList.push_back(vertex);
    }
    if (distanceToFace3 > m_epsilon) {
      face3->m_conflictList.push_back(vertex);
    }
    if (distanceToFace4 > m_epsilon) {
      face4->m_conflictList.push_back(vertex);
    }
  }
  assert(m_hullHalEdgeFacesPtr.size() == 4);
  assert(m_halfEdgeVertices.size() == 4);
  assert(m_hullHalfEdgesPtr.size() == 12);
#ifdef QUICK_HULL_DEBUG
  int index = 0;
  for (const QhFace *face : qhFaceHullSet) {
    index++;
    std::cout << "face " << index << std::endl;
    face->printVertices();
  }
  std::cout << "End of initial Hull" << std::endl;
#endif // QUICK_HULL_DEBUG
}

float QuickHull::calculateEpsilon() {
  float maxAbsX = 0.0;
  float maxAbsY = 0.0;
  float maxAbsZ = 0.0;
  for (const glm::vec3 &vertex : m_vertices) {
    float absX = glm::abs(vertex.x);
    maxAbsX = maxAbsX < absX ? absX : maxAbsX;
    float absY = glm::abs(vertex.y);
    maxAbsY = maxAbsY < absY ? absY : maxAbsY;
    float absZ = glm::abs(vertex.z);
    maxAbsZ = maxAbsZ < absZ ? absZ : maxAbsZ;
  }
  return 3 * (maxAbsX + maxAbsY + maxAbsZ) *
         std::numeric_limits<float>::epsilon();
}
std::list<HalfEdge *> QuickHull::getHorizon(HalfEdgeFace *face,
                                            const glm::vec3 &eye) {

  face->m_visited = true;

  std::list<HalfEdgeFace *> nextFaces;
  for (const HalfEdge *edge : face->getEdges()) {
    HalfEdgeFace *currentFace = edge->m_twin->m_face;

    if (!currentFace->m_visited) {
      float distance = currentFace->distanceToPoint(eye);
      if (distance > m_epsilon) {
        nextFaces.push_back(currentFace);
      }
    }
  }

  std::list<HalfEdge *> horizonEdges;
  for (HalfEdgeFace *face : nextFaces) {
    horizonEdges = QuickHull::getHorizon(face, eye);
  }
  for (HalfEdge *edge : face->getEdges()) {
    HalfEdgeFace *currentFace = edge->m_twin->m_face;

    if (!currentFace->m_visited) {
      horizonEdges.push_back(edge->m_twin);
    }
  }
  return horizonEdges;
}
} // namespace ph
