#include "Physics.h"
#include "glm/common.hpp"
#include "glm/ext/matrix_common.hpp"
#include "glm/geometric.hpp"
Particle::Particle(const glm::vec3 initialPosition,
                   const glm::vec3 initialVelocity,
                   const glm::vec3 initialForceAccumulator, const float mass)
    : m_mass(mass), m_position(initialPosition), m_velocity(initialVelocity),
      m_forceAcumulator(initialForceAccumulator) {}
ParticleDerivative::ParticleDerivative(const glm::vec3 dPosition,
                                       const glm::vec3 dVelocity)
    : m_dPosition(dPosition), m_dVelocity(dVelocity) {}
ParticleDerivative Particle::getDerivatives(glm::vec3 force) {
  m_forceAcumulator = force;
  return ParticleDerivative(m_velocity, m_forceAcumulator / m_mass);
}
void ParticleSystem::eulerStep(const float delta) {
  std::vector<Particle> nextParticles = m_particles;
  for (Particle &nextParticle : nextParticles) {

    nextParticle.m_forceAcumulator = glm::vec3(0.0, 0.0, 0.0);
    glm::vec3 dPosition = nextParticle.m_velocity;
    glm::vec3 gravity = glm::vec3(0.0f, 0.0f, -9.8f) * nextParticle.m_mass;
    glm::vec3 dVelocity =
        gravity + nextParticle.m_forceAcumulator / nextParticle.m_mass;
    dPosition *= delta;
    dVelocity *= delta;
    nextParticle.m_position += dPosition;
    nextParticle.m_velocity += dVelocity;
    m_simulationClock += delta;
  }
  uint32_t particleIndex = 0;
  const float deltaMaxError = 0.2;
  for (Particle &nextParticle : nextParticles) {
    for (Plane &plane : m_planes) {
      glm::vec3 nextPosition = nextParticle.m_position;
      glm::vec3 previousPosition = m_particles.at(particleIndex).m_position;
      float deltaError = glm::length(nextPosition - previousPosition)/2.0f;
      while(deltaError>deltaMaxError)
      {
      if (glm::dot(nextPosition - plane.m_point, plane.m_normal) < 0.0f) {

         nextPosition = glm::mix(nextPosition, previousPosition, 0.5f);
          deltaError/=2.0f;
      }
      }
    }
    particleIndex++;
  }
}
void ParticleDerivative::scale(const float scaleConstant) {
  m_dPosition = m_dPosition * scaleConstant;
}
Plane::Plane(glm::vec3 point, glm::vec3 normal)
    : m_point(point), m_normal(normal) {}
void ParticleSystem::addParticle(const Particle particle) {
  m_particles.push_back(particle);
}
void ParticleSystem::addPlane(const Plane plane) { m_planes.push_back(plane); }
