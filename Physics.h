#pragma once
#include "glm/glm.hpp"
#include <istream>
#include <vector>
namespace Physics {

class ParticleDerivative {
public:
  friend class ParticleSystem;
  ParticleDerivative() = delete;
  ParticleDerivative(const glm::vec3 &dPosition, const glm::vec3 &dVelocity);
  void scale(const float scaleConstant);

private:
  glm::vec3 m_dPosition;
  glm::vec3 m_dVelocity;
};
class Plane {
public:
  Plane() = delete;
  friend class ParticleSystem;
  Plane(glm::vec3 point, glm::vec3 normal);

private:
  glm::vec3 m_point;
  glm::vec3 m_normal;
};
class Particle {
public:
  friend class ParticleSystem;
  Particle() = delete;
  Particle(const glm::vec3 &initialPosition, const glm::vec3 &initialVelocity,
           const glm::vec3 &initialForceAccumulator, const float m_mass);
  ParticleDerivative getDerivatives(const glm::vec3 &force);
  glm::vec3 getPosition();

private:
  float m_mass;
  glm::vec3 m_position;
  glm::vec3 m_velocity;
  glm::vec3 m_forceAcumulator;
};
class ParticleSystem {
public:
  ParticleSystem() = default;
  void eulerStep(const float delta);
  void addParticle(const Particle &particle);
  void addPlane(const Plane &plane);
  glm::vec3 getParticlePosition(const uint32_t index);

private:
  float m_simulationClock;
  std::vector<Particle> m_particles;
  std::vector<Plane> m_planes;
};
class CollisionShape {
public:
  virtual ~CollisionShape();
  virtual bool collidesWith(const CollisionShape &otherShape) = 0;
  float findMinimalStep(const CollisionShape &otherShape, float minimumStep);

private:
  glm::vec3 centerOfMass = glm::vec3(0.0f);
};
} // namespace Physics
