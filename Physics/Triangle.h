#pragma once
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
  virtual ~Triangle()=default;
  Triangle(const Triangle &triangle) = default;
  const Triangle &operator=(const Triangle &triangle);
  Triangle(const glm::vec3 &vertex1, const glm::vec3 &vertex2,
           const glm::vec3 &vertex3);
  Triangle getOpositeHandednesTriangle() const;
  bool isTowards(const glm::vec3 &point, const float &epsilon = 0.0f) const;
  float getDistanceToPointFromPlane(const glm::vec3 &point) const;
  std::vector<glm::vec3>
  getAllPointTowards(const std::vector<glm::vec3> &points) const;
  std::optional<glm::vec3>
  getFurthestPointTowards(const std::vector<glm::vec3> &points) const;
  glm::vec3 getFarthestPoint(const std::vector<glm::vec3> &points) const;
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
} // namespace ph
