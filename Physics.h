#pragma once
#include "glm/fwd.hpp"
#include "glm/glm.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"
#include <array>
#include <istream>
#include <sys/wait.h>
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

private:
};
struct State {
  glm::vec3 m_velocity;
  glm::vec3 m_angularVelocity;
  glm::vec3 m_forces;
  glm::vec3 m_torques;
};
class RigidBody {
public:
  RigidBody(float mass, glm::mat3 Ibody, glm::vec3 initialPosition,
            glm::quat initialOrientation, glm::vec3 initialVelocity, glm::vec3 initialAngularMomentum);
  State getDerivative(glm::vec3 forces, glm::vec3 torques);
  void eulerStep(float delta);
  void addForcesAndTorques(glm::vec3 force, glm::vec3 torque);
  void clearForcesAndTorques();
  glm::vec3 getPosition();
  glm::quat getOrientation();

private:
  const float m_mass;
  const glm::mat3 m_Ibody;
  const glm::mat3 m_IbodyInv;
  // state variables
  glm::vec3 m_position;
  glm::quat m_orientation;
  glm::vec3 m_linearMomentum;
  glm::vec3 m_angularMomentum;
  // derived quantities
  glm::mat3 getInvInertialTensor();
  glm::mat3 getInertialTensor();
  glm::vec3 getVelocity();
  glm::vec3 getAngularVelocity();
  // computed quantities
  glm::vec3 m_force = glm::vec3(0.0f);
  glm::vec3 m_torque = glm::vec3(0.0f);

  float m_time = 0.0f;
};
glm::mat3 star(glm::vec3 a);
class RigidBodySystem {
public:
  void eulerStep(float delta);
  void addRigidBody(RigidBody *rigidBody);

private:
  std::vector<RigidBody *> m_rigidBodies;
};
} // namespace Physics
