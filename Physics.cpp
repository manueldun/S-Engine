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

void ParticleSystem::eulerStep(const float time) {
  static float lastTime = time;
  static float delta = time - lastTime;
  delta = time - lastTime;
  const float MAX_DELTA = 0.016f;
  if (delta > MAX_DELTA) {
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
    const float deltaMaxError = 0.002;
    for (Particle &nextParticle : nextParticles) {
      for (Plane &plane : m_planes) {

        glm::vec3 nextPosition = nextParticle.m_position;

        if (glm::dot(nextPosition - plane.m_point, plane.m_normal) < 0.0f) {

          nextPosition = glm::mix(
              nextPosition, m_particles.at(particleIndex).m_position, 0.5f);
          float distanceError =
              glm::dot(nextPosition - plane.m_point, plane.m_normal);

          while (distanceError >= deltaMaxError && distanceError <= 0.0f) {

            if (distanceError < 0.0f) {

              nextPosition = glm::mix(
                  nextPosition, m_particles.at(particleIndex).m_position, 0.5f);

            } else {
              nextPosition = glm::mix(
                  nextPosition, m_particles.at(particleIndex).m_position, 1.5f);
            }
            distanceError =
                glm::dot(nextPosition - plane.m_point, plane.m_normal);
          }

          nextParticle.m_velocity = -nextParticle.m_velocity * 0.5f;
        }
        nextParticle.m_position = nextPosition;
      }

      m_particles = nextParticles;

      particleIndex++;
    }
    delta -= MAX_DELTA;
    lastTime = time;
  }
}

void ParticleSystem::addParticle(const Particle &particle) {
  m_particles.push_back(particle);
}

void ParticleSystem::addPlane(const Plane &plane) { m_planes.push_back(plane); }

glm::vec3 ParticleSystem::getParticlePosition(const uint32_t index) {
  return m_particles.at(index).m_position;
}
RigidBody::RigidBody(float mass, glm::mat3 Ibody, glm::vec3 initialPosition,
                     glm::mat3 initialOrientation, glm::vec3 initialVelocity,
                     glm::vec3 initialAngularVelocity)
    : m_mass(mass), m_Ibody(Ibody), m_IbodyInv(glm::inverse(Ibody)),
      m_position(initialPosition), m_orientation(initialOrientation),

      m_Iinv(initialOrientation * m_IbodyInv *
             glm::transpose(initialOrientation)),

      m_velocity(initialVelocity), m_angularVelocity(initialAngularVelocity) {}
State Physics::RigidBody::getDerivative(glm::vec3 forces, glm::vec3 torques) {
  State derivativeState = {};
  derivativeState.m_position = m_velocity;
  derivativeState.m_orientation = star(m_angularVelocity) * m_orientation;
  derivativeState.m_linearMomentum = forces;
  derivativeState.m_angularMomentum = torques;
  return derivativeState;
}
void RigidBody::eulerStep(float delta) {
  State derivativeState = getDerivative(m_force, m_torque);
}
void RigidBody::addForcesAndTorques(glm::vec3 force, glm::vec3 torque) {
  m_force += force;
  m_torque += torque;
}
glm::mat3 star(glm::vec3 a) {
  return glm::mat3(glm::vec3(0.0f, -a.z, a.y), glm::vec3(a.z, 0.0f, -a.x),
                   glm::vec3(-a.y, a.x, 0.0f));
}
} // namespace Physics
