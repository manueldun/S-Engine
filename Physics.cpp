#include "Physics.h"
#include "glm/common.hpp"
#include "glm/geometric.hpp"
#include "glm/gtc/quaternion.hpp"
#include <iostream>
#include <stdexcept>
#include <sys/wait.h>
namespace Physics {
ParticleDerivative::ParticleDerivative(const glm::vec3 &dPosition,
                                       const glm::vec3 &dVelocity)
    : m_dPosition(dPosition), m_dVelocity(dVelocity) {}

void ParticleDerivative::scale(const float &scaleConstant) {
  m_dPosition = m_dPosition * scaleConstant;
}

Plane::Plane(const glm::vec3 &point, const glm::vec3 &normal)
    : m_point(point), m_normal(normal) {}

Particle::Particle(const glm::vec3 &initialPosition,
                   const glm::vec3 &initialVelocity,
                   const glm::vec3 &initialForceAccumulator, const float &mass)
    : m_mass(mass), m_position(initialPosition), m_velocity(initialVelocity),
      m_forceAcumulator(initialForceAccumulator) {}

const ParticleDerivative Particle::getDerivatives(const glm::vec3 &force) {
  m_forceAcumulator = force;
  return ParticleDerivative(m_velocity, m_forceAcumulator / m_mass);
}

glm::vec3 Particle::getPosition() { return m_position; }

void ParticleSystem::eulerStep(const float &time) {
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

      m_particles = std::move(nextParticles);

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

glm::vec3 ParticleSystem::getParticlePosition(const uint32_t &index) const {
  return m_particles.at(index).m_position;
}
RigidBody::RigidBody(const float &mass, const glm::mat3 &Ibody,
                     const glm::vec3 &initialPosition,
                     const glm::quat &initialOrientation,
                     const glm::vec3 &initialVelocity,
                     const glm::vec3 &initialAngularVelocity)
    : m_mass(mass), m_Ibody(Ibody), m_IbodyInv(glm::inverse(Ibody)),
      m_position(initialPosition), m_orientation(initialOrientation),
      m_linearMomentum(initialVelocity * m_mass),
      m_angularMomentum(Ibody * initialAngularVelocity)

{}
RigidBody::RigidBody(const float &mass, const glm::mat3 &Ibody,
                     const glm::vec3 &initialPosition,
                     const glm::quat &initialOrientation,
                     const glm::vec3 &initialVelocity,
                     const glm::vec3 &initialAngularVelocity,
                     const std::span<glm::vec3> &vertices)
    : m_mass(mass), m_Ibody(Ibody), m_IbodyInv(glm::inverse(Ibody)),
      m_position(initialPosition), m_orientation(initialOrientation),
      m_linearMomentum(initialVelocity * m_mass),
      m_angularMomentum(Ibody * initialAngularVelocity),m_vertices(vertices)

{}
const State Physics::RigidBody::getDerivative(const glm::vec3 &forces,
                                              const glm::vec3 &torques) const {
  State derivativeState = {};
  derivativeState.m_velocity = getVelocity();
  derivativeState.m_angularVelocity = getAngularVelocity();
  derivativeState.m_forces = forces + glm::vec3(0.0f, 0.0f, -1.0f) *
                                          RigidBodySystem::c_gravity * m_mass;
  derivativeState.m_torques = torques;
  return derivativeState;
}
void RigidBody::eulerStep(const float &delta) {
  State derivativeState = getDerivative(m_force, m_torque);
  m_position = m_position + delta * derivativeState.m_velocity;
  m_orientation =
      (0.5f * delta * derivativeState.m_angularVelocity) * m_orientation;
  m_linearMomentum = m_linearMomentum + delta * derivativeState.m_forces;
  m_angularMomentum = m_angularMomentum + delta * derivativeState.m_torques;
  m_time += delta;
}
void RigidBody::addForcesAndTorques(const glm::vec3 &force,
                                    const glm::vec3 &torque) {
  m_force += force;
  m_torque += torque;
}
void RigidBody::clearForcesAndTorques() {
  m_force = glm::vec3(0.0f);
  m_torque = glm::vec3(0.0f);
}
glm::vec3 RigidBody::getPosition() const { return m_position; }

glm::quat RigidBody::getOrientation() const { return m_orientation; }
bool RigidBody::doesIntersect(const RigidBody &rigidBody) const {
  throw std::runtime_error("intersection test not implemented yet");
  return false;
}
glm::mat3 RigidBody::getInertialTensor() const {
  glm::mat3 orientation = glm::mat3_cast(m_orientation);
  return orientation * m_Ibody * glm::transpose(orientation);
}
glm::mat3 RigidBody::getInvInertialTensor() const {
  glm::mat3 orientation = glm::mat3_cast(m_orientation);
  return orientation * m_IbodyInv * glm::transpose(orientation);
}
glm::vec3 RigidBody::getVelocity() const { return m_linearMomentum / m_mass; }
glm::vec3 RigidBody::getAngularVelocity() const {
  return (getInvInertialTensor() * m_angularMomentum);
}
void RigidBodySystem::eulerStep(const float &delta) {

  for (RigidBody *rigidBody : m_rigidBodies) {
    rigidBody->addForcesAndTorques(glm::vec3(0.0f, 0.0f, 0.0f),
                                   glm::vec3(0.0f));
    rigidBody->eulerStep(delta);
    rigidBody->clearForcesAndTorques();
  }
}
void RigidBodySystem::addRigidBody(RigidBody *rigidBody) {
  m_rigidBodies.push_back(rigidBody);
}
glm::mat3 star(const glm::vec3 &a) {
  return glm::mat3(glm::vec3(0.0f, -a.z, a.y), glm::vec3(a.z, 0.0f, -a.x),
                   glm::vec3(-a.y, a.x, 0.0f));
}
} // namespace Physics
