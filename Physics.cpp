#include "Physics.h"
#include "glm/common.hpp"
#include "glm/geometric.hpp"
#include <iostream>
#include <stdexcept>
namespace Physics {
ParticleDerivative::ParticleDerivative(const glm::vec3 &dPosition,
                                       const glm::vec3 &dVelocity)
    : m_dPosition(dPosition), m_dVelocity(dVelocity) {}

void ParticleDerivative::scale(const float scaleConstant) {
  m_dPosition = m_dPosition * scaleConstant;
}

Plane::Plane(glm::vec3 point, glm::vec3 normal)
    : m_point(point), m_normal(normal) {}

Particle::Particle(const glm::vec3 &initialPosition,
                   const glm::vec3 &initialVelocity,
                   const glm::vec3 &initialForceAccumulator, const float mass)
    : m_mass(mass), m_position(initialPosition), m_velocity(initialVelocity),
      m_forceAcumulator(initialForceAccumulator) {}

ParticleDerivative Particle::getDerivatives(const glm::vec3 &force) {
  m_forceAcumulator = force;
  return ParticleDerivative(m_velocity, m_forceAcumulator / m_mass);
}

glm::vec3 Particle::getPosition() { return m_position; }

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
  const float DELTA_MAX_ERROR = 0.002;
  for (Particle &nextParticle : nextParticles) {
    for (Plane &plane : m_planes) {

      glm::vec3 nextPosition = nextParticle.m_position;

      if (glm::dot(nextPosition - plane.m_point, plane.m_normal) < 0.0f) {

        nextPosition = glm::mix(nextPosition,
                                m_particles.at(particleIndex).m_position, 0.5f);
        float distanceError =
            glm::dot(nextPosition - plane.m_point, plane.m_normal);

        while (distanceError >= DELTA_MAX_ERROR && distanceError <= 0.0f) {

          if (distanceError < 0.0f) {

            nextPosition = glm::mix(
                nextPosition, m_particles.at(particleIndex).m_position, 0.5f);

            std::cout << "Backing off, Error:" << distanceError << std::endl;
          } else {
            nextPosition = glm::mix(
                nextPosition, m_particles.at(particleIndex).m_position, 1.5f);

            std::cout << "Going forward, Error:" << distanceError << std::endl;
          }
          distanceError =
              glm::dot(nextPosition - plane.m_point, plane.m_normal);
        }

        nextParticle.m_velocity = -nextParticle.m_velocity * 0.8f;
      }
      nextParticle.m_position = nextPosition;
    }

    m_particles = nextParticles;

    particleIndex++;
  }
}

void ParticleSystem::addParticle(const Particle &particle) {
  m_particles.push_back(particle);
}

void ParticleSystem::addPlane(const Plane &plane) { m_planes.push_back(plane); }

float CollisionShape::findMinimalStep(const CollisionShape &otherShape,
                                      float minimumStep) {
  throw std::runtime_error("findMinimalStep() not implemented yet");
  float distance = glm::length(this->centerOfMass - otherShape.centerOfMass);
  const int MAX_ITERATIONS = 100;
  int iterations = 0;
  do {
    iterations++;
    if (iterations >= MAX_ITERATIONS) {
      throw std::runtime_error("more than 100 iterations in findMinimalStep()");
    }
    distance = glm::length(this->centerOfMass - otherShape.centerOfMass);
  } while (minimumStep > distance);
  return distance;
}
glm::vec3 ParticleSystem::getParticlePosition(const uint32_t index) {
  return m_particles.at(index).m_position;
}
} // namespace Physics
