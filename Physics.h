#pragma once
#include "Renderer.h"
#include "glm/ext/vector_float3.hpp"
#include "glm/fwd.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"
#include <span>
#include <sys/wait.h>
#include <vector>
namespace Physics {

class ParticleDerivative {
public:
  friend class ParticleSystem;
  ParticleDerivative() = delete;
  ParticleDerivative(const glm::vec3 &dPosition, const glm::vec3 &dVelocity);
  void scale(const float &scaleConstant);

private:
  glm::vec3 m_dPosition;
  glm::vec3 m_dVelocity;
};
class Plane {
public:
  Plane() = delete;
  friend class ParticleSystem;
  Plane(const glm::vec3 &point, const glm::vec3 &normal);

private:
  const glm::vec3 m_point;
  const glm::vec3 m_normal;
};
class Particle {
public:
  friend class ParticleSystem;
  Particle() = delete;
  Particle(const glm::vec3 &initialPosition, const glm::vec3 &initialVelocity,
           const glm::vec3 &initialForceAccumulator, const float &m_mass);
  const ParticleDerivative getDerivatives(const glm::vec3 &force);
  glm::vec3 getPosition();

private:
  const float m_mass;
  glm::vec3 m_position;
  glm::vec3 m_velocity;
  glm::vec3 m_forceAcumulator;
};
class ParticleSystem {
public:
  void eulerStep(const float &delta);
  void addParticle(const Particle &particle);
  void addPlane(const Plane &plane);
  glm::vec3 getParticlePosition(const uint32_t &index) const;

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
  RigidBody(const float &mass, const glm::mat3 &Ibody,
            const glm::vec3 &initialPosition,
            const glm::quat &initialOrientation,
            const glm::vec3 &initialVelocity,
            const glm::vec3 &initialAngularVelocity);
  RigidBody(
      const std::span<const glm::vec3> &vertices, const float &mass,
      const glm::mat3 &Ibody = glm::mat3(0.0f),
      const glm::vec3 &initialPosition = glm::vec3(0.0f),
      const glm::quat &initialOrientation = glm::quat(0.0f, 0.0f, 0.0f, 1.0f),
      const glm::vec3 &initialVelocity = glm::vec3(0.0f),
      const glm::vec3 &initialAngularVelocity = glm::vec3(0.0f));
  const State getDerivative(const glm::vec3 &forces,
                            const glm::vec3 &torques) const;
  void eulerStep(const float &delta);
  void addForcesAndTorques(const glm::vec3 &force, const glm::vec3 &torque);
  void clearForcesAndTorques();
  glm::vec3 getPosition() const;
  glm::quat getOrientation() const;
  bool doesIntersect(const RigidBody &rigidBody) const;

private:
  const std::span<const glm::vec3> m_vertices;
  const float m_mass;
  const glm::mat3 m_Ibody;
  const glm::mat3 m_IbodyInv;
  // state variables
  glm::vec3 m_position;
  glm::quat m_orientation;
  glm::vec3 m_linearMomentum;
  glm::vec3 m_angularMomentum;
  // derived quantities
  glm::mat3 getInertialTensor() const;
  glm::mat3 getInvInertialTensor() const;
  glm::vec3 getVelocity() const;
  glm::vec3 getAngularVelocity() const;
  // computed quantities
  glm::vec3 m_force = glm::vec3(0.0f);
  glm::vec3 m_torque = glm::vec3(0.0f);

  float m_time = 0.0f;

};
class RigidBodySystem {
public:
  void eulerStep(const float &delta);
  void addRigidBody(const RigidBody &rigidBody);
  RigidBody &addMesh(const tinygltf::Model &model, const float &mass,
                     const glm::vec3 &initialPosition = glm::vec3(0.0f),
                     const glm::quat &initialOrientation = glm::quat(0.0f, 0.0f,
                                                                     0.0f,
                                                                     1.0f),
                     const glm::vec3 &initialVelocity = glm::vec3(0.0f),
                     const glm::vec3 &initialAngularVelocity = glm::vec3(0.0f));

  static constexpr float c_gravity = 9.8f;

private:
  std::vector<RigidBody> m_rigidBodies;
};
glm::mat3 star(const glm::vec3 &a);
} // namespace Physics
