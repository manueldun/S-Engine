#pragma once
#include "glm/glm.hpp"
#include <vector>
class ParticleDerivative {
public:
  friend class ParticleSystem;
  ParticleDerivative() = delete;
  ParticleDerivative(const glm::vec3 dPosition, const glm::vec3 dVelocity);
  void scale(const float scaleConstant);

private:
  glm::vec3 m_dPosition;
  glm::vec3 m_dVelocity;
};
class Particle {
public:
  friend class ParticleSystem;
  Particle() = delete;
  Particle(const glm::vec3 initialPosition, const glm::vec3 initialVelocity,
           const glm::vec3 initialForceAccumulator, const float m_mass);
  ParticleDerivative getDerivatives(glm::vec3 force);
  void step(const float stepSize);

private:
  float m_mass;
  glm::vec3 m_position;
  glm::vec3 m_velocity;
  glm::vec3 m_forceAcumulator;
};
class Plane{
public:
  friend class ParticleSystem;
  Plane(glm::vec3 point,glm::vec3 normal);
private:
  glm::vec3 m_point;
  glm::vec3 m_normal;
};
class ParticleSystem {
public:
  ParticleSystem() = default;
  void eulerStep(const float delta);
  void addParticle(const Particle particle);
  void addPlane(const Plane plane);
private:
  float m_simulationClock;
  std::vector<Particle> m_particles;
  std::vector<Plane> m_planes;
};
class Physics {
public:
private:
};
