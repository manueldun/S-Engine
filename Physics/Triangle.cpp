#include "Triangle.h"

namespace ph {
glm::vec3 farthestPointFromLine(const glm::vec3 &linePoint1,
                                const glm::vec3 &linePoint2,
                                const std::vector<glm::vec3> &points) {
  glm::vec3 farthestPoint = points.at(0);
  glm::vec3 numeratorVec =
      glm::cross(farthestPoint - linePoint1, farthestPoint - linePoint2);
  glm::vec3 denominatorVec = linePoint2 - linePoint1;
  float maxDistance = glm::length(numeratorVec) / glm::length(denominatorVec);
  for (const glm::vec3 point : points) {
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
float Triangle::getDistanceToPointFromPlane(const glm::vec3 &point) const {
  const glm::vec3 edge1 = m_vertex2 - m_vertex1;
  const glm::vec3 edge2 = m_vertex3 - m_vertex1;
  const glm::vec3 crossVector = glm::cross(edge1, edge2);
  const glm::vec3 normal = glm::normalize(crossVector);
  return glm::dot(normal, point - m_vertex1);
}
std::vector<glm::vec3>
Triangle::getAllPointTowards(const std::vector<glm::vec3> &points) const {
  std::vector<glm::vec3> overPoints;

  for (const glm::vec3 &point : points) {
    if (isTowards(point)) {
      overPoints.push_back(point);
    }
  }
  return overPoints;
}
std::optional<glm::vec3>
Triangle::getFurthestPointTowards(const std::vector<glm::vec3> &points) const {
  std::optional<glm::vec3> furthest;
  for (const glm::vec3 &point : points) {
    float distanceFromPoint = getDistanceToPointFromPlane(point);
    if (distanceFromPoint > 0.0) {
      furthest = point;
    }
    if (furthest.has_value()) {
      if (getDistanceToPointFromPlane(point) >
          getDistanceToPointFromPlane(furthest.value())) {
        furthest = point;
      }
    }
  }
  return furthest;
}

glm::vec3
Triangle::getFarthestPoint(const std::vector<glm::vec3> &points) const {
  glm::vec3 furthest = points.at(0);

  float maxDistance = getDistanceToPointFromPlane(points.at(0));
  for (const glm::vec3 &point : points) {

    float distanceFromPoint = glm::abs(getDistanceToPointFromPlane(point));
    if (maxDistance < distanceFromPoint) {
      maxDistance = distanceFromPoint;
      furthest = point;
    }
  }
  return furthest;
}
glm::vec3 Triangle::getNormal() const {

  const glm::vec3 triangleEdge1 = m_vertex2 - m_vertex1;
  const glm::vec3 triangleEdge2 = m_vertex3 - m_vertex1;
  return glm::normalize(glm::cross(triangleEdge1, triangleEdge2));
}
const Triangle &Triangle::operator=(const Triangle &triangle) {
  return triangle;
}
Triangle::Triangle(const glm::vec3 &vertex1, const glm::vec3 &vertex2,
                   const glm::vec3 &vertex3)
    : m_vertex1(vertex1), m_vertex2(vertex2), m_vertex3(vertex3) {}

Triangle Triangle::getOpositeHandednesTriangle() const {

  return Triangle(m_vertex2, m_vertex1, m_vertex3);
}
Triangle Triangle::transform(const glm::mat4 &transform) const {
  return Triangle(transform * glm::vec4(m_vertex1, 1.0f),
                  transform * glm::vec4(m_vertex2, 1.0f),
                  transform * glm::vec4(m_vertex3, 1.0f));
}
bool Triangle::isTowards(const glm::vec3 &point, const float &epsilon) const {
  const glm::vec3 triangleEdge1 = m_vertex2 - m_vertex1;
  const glm::vec3 triangleEdge2 = m_vertex3 - m_vertex1;
  const glm::vec3 normal =
      glm::normalize(glm::cross(triangleEdge1, triangleEdge2));
  const float dotProduct = glm::dot(normal, point - m_vertex1);
  return dotProduct - epsilon > 0.0f;
}
} // namespace ph
