#pragma once
#include <array>
#include <glm/glm.hpp>
#include <optional>
#include <vector>
namespace ph {

glm::vec3 farthestPointFromLine(const glm::vec3 &linePoint1,
                                const glm::vec3 &linePoint2,
                                const std::vector<glm::vec3> &points);

enum EdgeId { FIRST, SECOND, THIRD };
class Triangle {
public:
  Triangle() = default;
  virtual ~Triangle() = default;
  Triangle(const Triangle &triangle) = default;
  Triangle(const glm::vec3 &vertex1, const glm::vec3 &vertex2,
           const glm::vec3 &vertex3);
  Triangle getOpositeHandednesTriangle() const;
  Triangle transform(const glm::mat4 &tranform) const;
  bool isTowards(const glm::vec3 &point, const float &epsilon = 0.0f) const;
  float getDistanceToPointFromPlane(const glm::vec3 &point) const;
  std::vector<glm::vec3>
  getAllPointTowards(const std::vector<glm::vec3> &points) const;
  std::optional<glm::vec3>
  getFurthestPointTowards(const std::vector<glm::vec3> &points) const;
  glm::vec3 getFarthestPoint(const std::vector<glm::vec3> &points) const;
  glm::vec3 getNormal() const;
  std::array<glm::vec3, 2> getEdge(const EdgeId &edgeId);
  float getVerticesElipson() const;
  glm::vec3 getVectorWiseMax() const;
  std::array<glm::vec3, 3> getVertexData() const;

private:
  const glm::vec3 m_vertex1 = glm::vec3(0.0f);
  const glm::vec3 m_vertex2 = glm::vec3(0.0f);
  const glm::vec3 m_vertex3 = glm::vec3(0.0f);
};
} // namespace ph
