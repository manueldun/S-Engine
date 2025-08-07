#pragma once
#include "Renderer.h"
#include "glm/ext/vector_float3.hpp"
#include "glm/fwd.hpp"
#include <cstddef>
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"
#include <sys/wait.h>
#include <vector>
#include "Triangle.h"
namespace ph {

struct State {
  glm::vec3 m_velocity;
  glm::vec3 m_angularVelocity;
  glm::vec3 m_forces;
  glm::vec3 m_torques;
};
class RigidBody {
public:
  RigidBody(const std::vector<glm::vec3> &vertices,
            const std::vector<size_t> &indexBuffer, const float &mass,
            const glm::mat3 &Ibody = glm::mat3(0.0f),
            const glm::vec3 &initialPosition = glm::vec3(0.0f),
            const glm::quat &initialOrientation =
                glm::rotate(glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                            glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
            const glm::vec3 &initialVelocity = glm::vec3(0.0f),
            const glm::vec3 &initialAngularVelocity = glm::vec3(0.0f));
  const State getDerivative(const glm::vec3 &forces,
                            const glm::vec3 &torques) const;
  void eulerStep(const float &delta);
  void addForcesAndTorques(const glm::vec3 &force, const glm::vec3 &torque);
  void clearForcesAndTorques();
  glm::vec3 getPosition() const;
  void setPosition(const glm::vec3 &position);
  glm::quat getOrientation() const;
  void setOrientation(const glm::quat &orientation);
  bool doesIntersect(const RigidBody &rigidBody) const;
  const glm::mat4 getTransform() const;

  struct PlanePointCollision {
    glm::vec3 point;
    Triangle trianglePlane;
  };
  static std::vector<PlanePointCollision> planePointCollisions(
      const std::vector<glm::vec3> &hullPoints1,
      const std::vector<size_t> &hullIndices1, const glm::mat4 &tranform1,
      const std::vector<glm::vec3> &hullPoints2,
      const std::vector<size_t> &hullIndices2, const glm::mat4 &tranform2);

  const std::vector<glm::vec3> vertices;
  const std::vector<size_t> indices;

private:
  const glm::quat getStandardOrientation() const;
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
class Body;

class RigidBodySystem {
public:
  void eulerStep(const float &delta);
  void addRigidBody(const RigidBody &rigidBody);
  Body addMesh(const tinygltf::Model &model, const float &mass,
               const glm::vec3 &initialPosition = glm::vec3(0.0f),
               const glm::quat &initialOrientation = glm::quat(1.0f, 0.0f, 0.0f,
                                                               0.0f),
               const glm::vec3 &initialVelocity = glm::vec3(0.0f),
               const glm::vec3 &initialAngularVelocity = glm::vec3(0.0f));

  Body addMesh(const Engine::MeshNode &meshNode, const float &mass = 0.0f,
               const glm::vec3 &initialPosition = glm::vec3(0.0f),
               const glm::quat &initialOrientation = glm::quat(1.0f, 0.0f, 0.0f,
                                                               0.0f),
               const glm::vec3 &initialVelocity = glm::vec3(0.0f),
               const glm::vec3 &initialAngularVelocity = glm::vec3(0.0f));
  const glm::vec3 getPosition(const uint32_t index);
  const glm::mat4 getTransform(const uint32_t index);

  static constexpr float c_gravity = 9.8f;

private:
  std::vector<RigidBody> m_rigidBodies;
};
class Body {
public:
  Body(const uint32_t index, RigidBodySystem &system);
  glm::vec3 getPosition() const;
  glm::mat4 getTransform() const;

private:
  const uint32_t m_index;
  RigidBodySystem &m_system;
};
} // namespace Physics
