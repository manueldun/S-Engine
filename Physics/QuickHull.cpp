#include "QuickHull.h"
#include "glm/common.hpp"
#include <iostream>
#include <map>
#include <sys/wait.h>

#ifdef QUICK_HULL_DEBUG
#include <iostream>
#endif // QUICK_HULL_DEBUG
namespace ph {
bool QhFace::isTowardsPoint(const glm::vec3 &point,
                            const float &epsilon) const {
  glm::vec3 point1 = edge->tail->position;
  glm::vec3 point2 = edge->next->tail->position;
  glm::vec3 point3 = edge->next->next->tail->position;
  Triangle triangle(point1, point2, point3);
  return triangle.isTowards(point, epsilon);
}
float QhFace::distanceToPoint(const glm::vec3 &point) const {

  glm::vec3 point1 = edge->tail->position;
  glm::vec3 point2 = edge->next->tail->position;
  glm::vec3 point3 = edge->next->next->tail->position;
  Triangle triangle(point1, point2, point3);
  return triangle.getDistanceToPointFromPlane(point);
}

glm::vec3 QhFace::getFurtherVertex() const {
  glm::vec3 point1 = edge->tail->position;
  glm::vec3 point2 = edge->next->tail->position;
  glm::vec3 point3 = edge->next->next->tail->position;
  Triangle triangle(point1, point2, point3);
  glm::vec3 further = conflictList.front();

  float maxDistance = triangle.getDistanceToPointFromPlane(further);
  for (const glm::vec3 &vertex : conflictList) {

    float distance = triangle.getDistanceToPointFromPlane(vertex);
    if (maxDistance < distance) {
      maxDistance = distance;
      further = vertex;
    }
  }
  return further;
}

void QhFace::printVertices() const {
  QhHalfEdge *currentEdge = edge;
  QhHalfEdge *initialEdge = edge;
  do {
    std::cout << "v " << currentEdge->tail->position.x << " "
              << currentEdge->tail->position.y << " "
              << currentEdge->tail->position.z << std::endl;
    currentEdge = currentEdge->next;
  } while (currentEdge != initialEdge);
}

std::list<QhHalfEdge *> QhFace::getEdges() const {
  QhHalfEdge *currentEdge = edge;
  QhHalfEdge *firstEdge = edge;
  std::list<QhHalfEdge *> edges;
  do {

    edges.push_back(currentEdge);
    currentEdge = currentEdge->next;
  } while (firstEdge != currentEdge);
  return edges;
}
glm::vec3 QhVertex::getFarthestFromLine(const glm::vec3 &linePoint1,
                                        const glm::vec3 &linePoint2,
                                        const std::list<glm::vec3> &qhVertices,
                                        const float &epsilon) {

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

glm::vec3 QhVertex::getFarthestTowardsTriangle(const glm::vec3 &vertex1,
                                               const glm::vec3 &vertex2,
                                               const glm::vec3 &vertex3,
                                               std::vector<glm::vec3> &vertices,
                                               const float &epsilon) {

  glm::vec3 furthest = vertices.front();

  float maxDistance = QhVertex::getDistancefromTriangle(
      vertex1, vertex2, vertex3, furthest, epsilon);
  for (glm::vec3 &qhVertex : vertices) {

    float distanceFromPoint = QhVertex::getDistancefromTriangle(
        vertex1, vertex2, vertex3, qhVertex, epsilon);
    if (maxDistance < distanceFromPoint) {
      maxDistance = distanceFromPoint;
      furthest = qhVertex;
    }
  }
  return furthest;
}
glm::vec3 QhVertex::getFarthestFromTriangle(
    const glm::vec3 &vertex1, const glm::vec3 &vertex2,
    const glm::vec3 &vertex3, const std::list<glm::vec3> &vertices,
    const float &epsilon) {

  glm::vec3 furthest = vertices.front();

  float maxDistance = QhVertex::getDistancefromTriangle(
      vertex1, vertex2, vertex3, furthest, epsilon);
  maxDistance = glm::abs(maxDistance);
  for (glm::vec3 qhVertex : vertices) {

    float distanceFromPoint = glm::abs(QhVertex::getDistancefromTriangle(
        vertex1, vertex2, vertex3, qhVertex, epsilon));
    if (maxDistance < distanceFromPoint) {
      maxDistance = distanceFromPoint;
      furthest = qhVertex;
    }
  }
  return furthest;
}
float QhVertex::getDistancefromTriangle(const glm::vec3 &vertex1,
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
    : vertices(vertices), epsilon(calculateEpsilon()), m_name(name) {}

void QuickHull::buildQuickHull() {

  buildInitialHull();
  int iteration = 1;
  printToObj(m_name + std::to_string(0));
  QhFace *nextFace = nullptr;
  do {
    nextFace = nullptr;
    for (QhFace *face : qhFaceHullSet) {
      if (face->conflictList.size() > 0) {
        nextFace = face;
        break;
      }
    }
    if (nextFace != nullptr) {
      glm::vec3 furthestVertex = nextFace->getFurtherVertex();
      std::cout << "#New Point: " << furthestVertex.x << "," << furthestVertex.y
                << "," << furthestVertex.z << "," << std::endl;
      std::list<QhHalfEdge *> horizon = getHorizon(nextFace, furthestVertex);
      assert(horizon.size() >= 3);
      std::cout << "#horizon:" << std::endl;
      for (QhHalfEdge *edge : horizon) {
        std::cout << "#(" << edge->tail->position.x << ","
                  << edge->tail->position.y << "," << edge->tail->position.z
                  << ")" << std::endl;
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
void QuickHull::createMesh() {
  std::map<QhVertex *, size_t> indicesMap;
  size_t index = 0;
  for (const QhFace &face : qhHullFaces) {
    std::list<QhHalfEdge*> edges = face.getEdges();
    assert(edges.size()==3);
    for(QhHalfEdge* edge:edges){
      if (!indicesMap.contains(edge->tail)) {
        indicesMap[edge->tail] = index;
        m_hullIndices.push_back(index);
        index++;
      }
      else{
        m_hullIndices.push_back(indicesMap.at(edge->tail));
      }
    }
  }
  for (const QhVertex &vertex : qhVertices) {
    m_hullVertices.push_back(vertex.position);
  }
}
void QuickHull::mergeHullToHorizon(std::list<QhHalfEdge *> horizon,
                                   const glm::vec3 &point) {
  QhHalfEdge *previousHalfEdge = nullptr;
  QhHalfEdge *firstHalfEdge = nullptr;
  QhHalfEdge *firstHalfEdgeTwin = nullptr;
  std::list<QhFace *> newFaces;
  qhVertices.emplace_back();
  qhVertices.back().position = point;
  std::vector<QhFace *> debugNewFaces;
  for (QhHalfEdge *horizonEdge : horizon) {
    qhHullFaces.emplace_back();
    QhFace *currentFace = &qhHullFaces.back();
    qhFaceHullSet.push_back(currentFace);
    debugNewFaces.push_back(currentFace);
    newFaces.push_back(currentFace);
    qhHalfHullEdges.emplace_back();
    QhHalfEdge *edge1 = &qhHalfHullEdges.back();
    qhHalfHullEdges.emplace_back();
    currentFace->edge = edge1;
    QhHalfEdge *edge2 = &qhHalfHullEdges.back();
    qhHalfHullEdges.emplace_back();
    QhHalfEdge *edge3 = &qhHalfHullEdges.back();
    horizonEdge->twin = edge1;
    edge1->twin = horizonEdge; // continue
    edge1->tail = horizonEdge->next->tail;
    edge2->tail = horizonEdge->tail;
    edge3->tail = &qhVertices.back();
    edge1->face = currentFace;
    edge2->face = currentFace;
    edge3->face = currentFace;
    edge1->next = edge2;
    edge2->next = edge3;
    edge3->next = edge1;
    edge1->previous = edge3;
    edge2->previous = edge1;
    edge3->previous = edge2;
    if (previousHalfEdge != nullptr) {
      edge3->twin = previousHalfEdge;
      previousHalfEdge->twin = edge3;
    }
    if (firstHalfEdge == nullptr) {
      firstHalfEdge = edge3;
    }
    previousHalfEdge = edge2;
    firstHalfEdgeTwin = edge2;
  }
  firstHalfEdge->twin = firstHalfEdgeTwin;
  firstHalfEdgeTwin->twin = firstHalfEdge;
  for (QhFace *face : debugNewFaces) {
    std::cout << "#New Face:" << std::endl;
      std::cout << "#(" << face->edge->tail->position.x << ","
                << face->edge->tail->position.y << "," << face->edge->tail->position.z
                << ")" << std::endl;
      std::cout << "#(" << face->edge->next->tail->position.x << ","
                << face->edge->next->tail->position.y << ","
                << face->edge->next->tail->position.z << ")" << std::endl;
      std::cout << "#(" << face->edge->next->next->tail->position.x << ","
                <<face->edge->next->next->tail->position.y << ","
                <<face->edge->next->next->tail->position.z << ")" << std::endl;

      std::cout << "\t#Twin 1:" << std::endl;
      for (QhHalfEdge *twinEdge : face->edge->twin->face->getEdges()) {
        std::cout << "\t#(" << twinEdge->tail->position.x << ","
                  << twinEdge->tail->position.y << ","
                  << twinEdge->tail->position.z << ")" << std::endl;
      }
      std::cout << "\t#Twin 2:" << std::endl;
      for (QhHalfEdge *twinEdge : face->edge->next->twin->face->getEdges()) {
        std::cout << "\t#(" << twinEdge->tail->position.x << ","
                  << twinEdge->tail->position.y << ","
                  << twinEdge->tail->position.z << ")" << std::endl;
      }
      std::cout << "\t#Twin 3:" << std::endl;
      for (QhHalfEdge *twinEdge : face->edge->next->next->twin->face->getEdges()) {
        std::cout << "\t#(" << twinEdge->tail->position.x << ","
                  << twinEdge->tail->position.y << ","
                  << twinEdge->tail->position.z << ")" << std::endl;
      }
  }

  std::vector<QhHalfEdge *> edgesToErase;
  for (auto it = qhFaceHullSet.begin(); it != qhFaceHullSet.end();) {
    QhFace *face = *it;
    if (face->visited) {
      QhHalfEdge *currentEdge = face->edge;
      QhHalfEdge *startEdge = face->edge;
      do {
        edgesToErase.push_back(currentEdge);
        currentEdge = currentEdge->next;

      } while (currentEdge != startEdge);
      it = qhFaceHullSet.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = qhHalfEdgeHullSet.begin(); it != qhHalfEdgeHullSet.end();) {
    for (QhHalfEdge *edge : edgesToErase) {
      if (edge == *it) {
        it = qhHalfEdgeHullSet.erase(it);
      } else {
        ++it;
      }
    }
  }
  for (QhFace *face : newFaces) {
    for (glm::vec3 &vertex : vertices) {
      float distance = face->distanceToPoint(vertex);
      if (distance > epsilon) {
        face->conflictList.push_back(vertex);
      }
    }
  }

#ifdef QUICK_HULL_DEBUG
  debugPrintdata();

#endif // QUICK_HULL_DEBUG
}

QhFace *QuickHull::getUnvisitedFace() {
  for (QhFace &face : qhHullFaces) {
    if (!face.visited) {
      return &face;
    }
  }
  return nullptr;
}
void QuickHull::printToObj(const std::string &name) {
  std::cout << "o " << name << std::endl;
  std::vector<std::vector<int>> faces;
  std::map<QhVertex *, int> indicesMap;
  std::vector<glm::vec3> vertices;
  static int vertexIndex = 1;
  for (const QhFace *face : qhFaceHullSet) {

    QhVertex *vertex = face->edge->tail;
    std::vector<int> faceIndices;
    if (indicesMap.contains(vertex)) {
      faceIndices.push_back(indicesMap.at(vertex));
    } else {
      faceIndices.push_back(vertexIndex);
      indicesMap[vertex] = vertexIndex;
      vertexIndex++;
      vertices.push_back(vertex->position);
    }
    vertex = face->edge->next->tail;
    if (indicesMap.contains(vertex)) {
      faceIndices.push_back(indicesMap.at(vertex));
    } else {
      faceIndices.push_back(vertexIndex);
      indicesMap[vertex] = vertexIndex;
      vertexIndex++;
      vertices.push_back(vertex->position);
    }
    vertex = face->edge->next->next->tail;
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

  glm::vec3 farthestXPositive = vertices.front();
  for (const glm::vec3 &vertex : vertices) {
    if (vertex.x > farthestXPositive.x) {
      farthestXPositive = vertex;
    }
  }
  glm::vec3 farthestXNegative = vertices.front();
  for (const glm::vec3 &vertex : vertices) {
    if (vertex.x < farthestXNegative.x) {
      farthestXNegative = vertex;
    }
  }
  glm::vec3 farthestYPositive = vertices.front();
  for (const glm::vec3 &vertex : vertices) {
    if (vertex.y > farthestYPositive.y) {
      farthestYPositive = vertex;
    }
  }
  glm::vec3 farthestYNegative = vertices.front();
  for (const glm::vec3 &vertex : vertices) {
    if (vertex.y < farthestYNegative.y) {
      farthestYNegative = vertex;
    }
  }
  glm::vec3 farthestZPositive = vertices.front();
  for (const glm::vec3 &vertex : vertices) {
    if (vertex.z > farthestZPositive.z) {
      farthestZPositive = vertex;
    }
  }
  glm::vec3 farthestZNegative = vertices.front();
  for (const glm::vec3 &vertex : vertices) {
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
    thirdVertex = QhVertex::getFarthestFromLine(firstVertex, secondVertex,
                                                qhVert, epsilon);
    fourthVertex = QhVertex::getFarthestFromTriangle(
        firstVertex, secondVertex, thirdVertex, qhVert, epsilon);

  } else if (maxY > maxX && maxY > maxZ) {
    firstVertex = farthestYPositive;
    secondVertex = farthestYNegative;
    std::list<glm::vec3> qhVert = {farthestXPositive, farthestXNegative,
                                   farthestZPositive, farthestZNegative};
    thirdVertex = QhVertex::getFarthestFromLine(firstVertex, secondVertex,
                                                qhVert, epsilon);
    fourthVertex = QhVertex::getFarthestFromTriangle(
        firstVertex, secondVertex, thirdVertex, qhVert, epsilon);
  } else {
    firstVertex = farthestZPositive;
    secondVertex = farthestZNegative;
    std::list<glm::vec3> qhVert = {farthestXPositive, farthestXNegative,
                                   farthestYPositive, farthestYNegative};
    thirdVertex = QhVertex::getFarthestFromLine(firstVertex, secondVertex,
                                                qhVert, epsilon);
    fourthVertex = QhVertex::getFarthestFromTriangle(
        firstVertex, secondVertex, thirdVertex, qhVert, epsilon);
  }

  qhVertices.emplace_back();
  qhVertices.back().position = firstVertex;
  QhVertex *firstQhVertex = &qhVertices.back();
  qhVertices.emplace_back();
  qhVertices.back().position = secondVertex;
  QhVertex *secondQhVertex = &qhVertices.back();
  qhVertices.emplace_back();
  qhVertices.back().position = thirdVertex;
  QhVertex *thirdQhVertex = &qhVertices.back();
  qhVertices.emplace_back();
  qhVertices.back().position = fourthVertex;
  QhVertex *fourthQhVertex = &qhVertices.back();
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

  qhHullFaces.emplace_back();
  QhFace *face1 = &qhHullFaces.back();
  qhHullFaces.emplace_back();
  QhFace *face2 = &qhHullFaces.back();
  qhHullFaces.emplace_back();
  QhFace *face3 = &qhHullFaces.back();
  qhHullFaces.emplace_back();
  QhFace *face4 = &qhHullFaces.back();

  qhFaceHullSet.push_back(face1);
  qhFaceHullSet.push_back(face2);
  qhFaceHullSet.push_back(face3);
  qhFaceHullSet.push_back(face4);

  QhVertex *face1FirstVertex = firstQhVertex;
  QhVertex *face1SecondVertex = secondQhVertex;
  QhVertex *face1ThirdVertex = thirdQhVertex;
  QhVertex *face1FourthVertex = fourthQhVertex;
  Triangle firstTriangle(face1FirstVertex->position,
                         face1SecondVertex->position,
                         face1ThirdVertex->position);
  if (firstTriangle.isTowards(face1FourthVertex->position)) {
    QhVertex *tempVert = face1FirstVertex;
    face1FirstVertex = face1SecondVertex;
    face1SecondVertex = tempVert;
  }
  // face1
  qhHalfHullEdges.emplace_back();
  QhHalfEdge *face1Edge1 = &qhHalfHullEdges.back();
  qhHalfEdgeHullSet.push_back(face1Edge1);
  face1Edge1->face = face1;
  face1Edge1->tail = face1FirstVertex;
  face1->edge = face1Edge1;

  qhHalfHullEdges.emplace_back();
  QhHalfEdge *face1Edge2 = &qhHalfHullEdges.back();
  qhHalfEdgeHullSet.push_back(face1Edge2);
  face1Edge2->face = face1;
  face1Edge2->tail = face1SecondVertex;

  qhHalfHullEdges.emplace_back();
  QhHalfEdge *face1Edge3 = &qhHalfHullEdges.back();
  qhHalfEdgeHullSet.push_back(face1Edge3);
  face1Edge3->face = face1;
  face1Edge3->tail = face1ThirdVertex;

  face1Edge1->next = face1Edge2;
  face1Edge2->next = face1Edge3;
  face1Edge3->next = face1Edge1;

  face1Edge1->previous = face1Edge3;
  face1Edge2->previous = face1Edge1;
  face1Edge3->previous = face1Edge2;

  QhVertex *face2FirstVertex = firstQhVertex;
  QhVertex *face2SecondVertex = secondQhVertex;
  QhVertex *face2ThirdVertex = thirdQhVertex;
  QhVertex *face2FourthVertex = fourthQhVertex;
  Triangle secondTriangle(face2SecondVertex->position,
                          face2ThirdVertex->position,
                          face2FourthVertex->position);
  if (secondTriangle.isTowards(face2FirstVertex->position)) {
    QhVertex *tempVert = face2SecondVertex;
    face2SecondVertex = face2ThirdVertex;
    face2ThirdVertex = tempVert;
  }
  // face2
  qhHalfHullEdges.emplace_back();
  QhHalfEdge *face2Edge1 = &qhHalfHullEdges.back();
  qhHalfEdgeHullSet.push_back(face2Edge1);
  face2Edge1->face = face2;
  face2Edge1->tail = face2SecondVertex;
  face2->edge = face2Edge1;

  qhHalfHullEdges.emplace_back();
  QhHalfEdge *face2Edge2 = &qhHalfHullEdges.back();
  qhHalfEdgeHullSet.push_back(face2Edge2);
  face2Edge2->face = face2;
  face2Edge2->tail = face2ThirdVertex;

  qhHalfHullEdges.emplace_back();
  QhHalfEdge *face2Edge3 = &qhHalfHullEdges.back();
  qhHalfEdgeHullSet.push_back(face2Edge3);
  face2Edge3->face = face2;
  face2Edge3->tail = face2FourthVertex;

  face2Edge1->next = face2Edge2;
  face2Edge2->next = face2Edge3;
  face2Edge3->next = face2Edge1;

  face2Edge1->previous = face2Edge3;
  face2Edge2->previous = face2Edge1;
  face2Edge3->previous = face2Edge2;

  QhVertex *face3FirstVertex = firstQhVertex;
  QhVertex *face3SecondVertex = secondQhVertex;
  QhVertex *face3ThirdVertex = thirdQhVertex;
  QhVertex *face3FourthVertex = fourthQhVertex;
  Triangle thirdTriangle(face3FirstVertex->position,
                         face3SecondVertex->position,
                         face3FourthVertex->position);
  if (thirdTriangle.isTowards(face3ThirdVertex->position)) {
    QhVertex *tempVert = face3FirstVertex;
    face3FirstVertex = face3SecondVertex;
    face3SecondVertex = tempVert;
  }
  // face3
  qhHalfHullEdges.emplace_back();
  QhHalfEdge *face3Edge1 = &qhHalfHullEdges.back();
  qhHalfEdgeHullSet.push_back(face3Edge1);
  face3Edge1->face = face3;
  face3Edge1->tail = face3FirstVertex;
  face3->edge = face3Edge1;

  qhHalfHullEdges.emplace_back();
  QhHalfEdge *face3Edge2 = &qhHalfHullEdges.back();
  qhHalfEdgeHullSet.push_back(face3Edge2);
  face3Edge2->face = face3;
  face3Edge2->tail = face3SecondVertex;

  qhHalfHullEdges.emplace_back();
  QhHalfEdge *face3Edge3 = &qhHalfHullEdges.back();
  qhHalfEdgeHullSet.push_back(face3Edge3);
  face3Edge3->face = face3;
  face3Edge3->tail = face3FourthVertex;

  face3Edge1->next = face3Edge2;
  face3Edge2->next = face3Edge3;
  face3Edge3->next = face3Edge1;

  face3Edge1->previous = face3Edge3;
  face3Edge2->previous = face3Edge1;
  face3Edge3->previous = face3Edge2;

  QhVertex *face4FirstVertex = firstQhVertex;
  QhVertex *face4SecondVertex = secondQhVertex;
  QhVertex *face4ThirdVertex = thirdQhVertex;
  QhVertex *face4FourthVertex = fourthQhVertex;
  Triangle fourthTriangle(face4FirstVertex->position,
                          face4ThirdVertex->position,
                          face4FourthVertex->position);
  if (fourthTriangle.isTowards(face4SecondVertex->position)) {
    QhVertex *tempVert = face4FirstVertex;
    face4FirstVertex = face4ThirdVertex;
    face4ThirdVertex = tempVert;
  }
  // face4
  qhHalfHullEdges.emplace_back();
  QhHalfEdge *face4Edge1 = &qhHalfHullEdges.back();
  qhHalfEdgeHullSet.push_back(face4Edge1);
  face4Edge1->face = face4;
  face4Edge1->tail = face4FirstVertex;
  face4->edge = face4Edge1;

  qhHalfHullEdges.emplace_back();
  QhHalfEdge *face4Edge2 = &qhHalfHullEdges.back();
  qhHalfEdgeHullSet.push_back(face4Edge2);
  face4Edge2->face = face4;
  face4Edge2->tail = face4ThirdVertex;

  qhHalfHullEdges.emplace_back();
  QhHalfEdge *face4Edge3 = &qhHalfHullEdges.back();
  qhHalfEdgeHullSet.push_back(face4Edge3);
  face4Edge3->face = face4;
  face4Edge3->tail = face4FourthVertex;

  face4Edge1->next = face4Edge2;
  face4Edge2->next = face4Edge3;
  face4Edge3->next = face4Edge1;

  face4Edge1->previous = face4Edge3;
  face4Edge2->previous = face4Edge1;
  face4Edge3->previous = face4Edge2;

  for (QhHalfEdge *edge1 : qhHalfEdgeHullSet) {

    for (QhHalfEdge *edge2 : qhHalfEdgeHullSet) {
      if (edge1->tail == edge2->next->tail &&
          edge1->next->tail == edge2->tail) {
        edge1->twin = edge2;
        edge2->twin = edge1;
      }
    }
  }

#ifdef QUICK_HULL_DEBUG
  debugPrintdata();
#endif // QUICK_HULL_DEBUG

  for (const glm::vec3 vertex : vertices) {
    float distanceToFace1 = face1->distanceToPoint(vertex);
    float distanceToFace2 = face2->distanceToPoint(vertex);
    float distanceToFace3 = face3->distanceToPoint(vertex);
    float distanceToFace4 = face4->distanceToPoint(vertex);
    if (distanceToFace1 > epsilon) {
      qhVertices.emplace_back();
      qhVertices.back().position = vertex;
      face1->conflictList.push_back(qhVertices.back().position);
    }
    if (distanceToFace2 > epsilon) {
      qhVertices.emplace_back();
      qhVertices.back().position = vertex;
      face2->conflictList.push_back(qhVertices.back().position);
    }
    if (distanceToFace3 > epsilon) {
      qhVertices.emplace_back();
      qhVertices.back().position = vertex;
      face3->conflictList.push_back(qhVertices.back().position);
    }
    if (distanceToFace4 > epsilon) {
      qhVertices.emplace_back();
      qhVertices.back().position = vertex;
      face4->conflictList.push_back(qhVertices.back().position);
    }
  }
  assert(qhHullFaces.size() == 4);
  assert(qhFaceHullSet.size() == 4);
  assert(qhHalfHullEdges.size() == 12);
  assert(qhHalfEdgeHullSet.size() == 12);
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

void QuickHull::deleteFace(QhFace *face) {
  for (auto it = qhHullFaces.begin(); it != qhHullFaces.end();) {
    if (&*it == face) {
      it = qhHullFaces.erase(it);
    } else {
      it++;
    }
  }
}
float QuickHull::calculateEpsilon() {
  float maxAbsX = 0.0;
  float maxAbsY = 0.0;
  float maxAbsZ = 0.0;
  for (const glm::vec3 &vertex : vertices) {
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
std::list<QhHalfEdge *> QuickHull::getHorizon(QhFace *face,
                                              const glm::vec3 &eye) {

  face->visited = true;

  std::list<QhFace *> nextFaces;
  for (const QhHalfEdge *edge : face->getEdges()) {
    QhFace *currentFace = edge->twin->face;

    if (!currentFace->visited) {
      float distance = currentFace->distanceToPoint(eye);
      if (distance > epsilon) {
        nextFaces.push_back(currentFace);
      }
    }
  }

  std::list<QhHalfEdge *> horizonEdges;
  for (QhFace *face : nextFaces) {
    horizonEdges = QuickHull::getHorizon(face, eye);
  }
  for (QhHalfEdge *edge : face->getEdges()) {
    QhFace *currentFace = edge->twin->face;

    if (!currentFace->visited) {
      horizonEdges.push_back(edge->twin);
    }
  }
  return horizonEdges;
}
} // namespace ph
