#include "Physics.h"
#include "QuickHull.h"
#include "glm/detail/qualifier.hpp"
#include "glm/exponential.hpp"
#include "glm/ext/matrix_transform.hpp"
#include "glm/ext/quaternion_geometric.hpp"
#include "glm/ext/quaternion_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include <iostream>
#include <sys/wait.h>
namespace ph {

RigidBody::RigidBody(const RigidBody &rigidBody)
    : RigidBody(rigidBody.m_hull, rigidBody.m_mass, rigidBody.m_Ibody,
                rigidBody.m_position, rigidBody.m_orientation,
                rigidBody.m_linearMomentum / rigidBody.m_mass,
                rigidBody.m_IbodyInv * rigidBody.m_angularMomentum) {}
RigidBody::RigidBody(const QuickHull &hull, const float &mass,
                     const glm::mat3 &Ibody, const glm::vec3 &initialPosition,
                     const glm::quat &initialOrientation,
                     const glm::vec3 &initialVelocity,
                     const glm::vec3 &initialAngularVelocity)
    : m_hull(hull), m_mass(mass), m_Ibody(Ibody),
      m_IbodyInv(glm::inverse(Ibody)), m_position(initialPosition),
      m_orientation(initialOrientation),
      m_linearMomentum(initialVelocity * m_mass),
      m_angularMomentum(Ibody * initialAngularVelocity) {}
const State RigidBody::getDerivative(const glm::vec3 &forces,
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
  if (m_mass != 0.0f) {
    m_position = m_position + delta * derivativeState.m_velocity;
    glm::vec4 w(0.0f, derivativeState.m_angularVelocity);
    glm::quat orientationDerivative = glm::quat(0.5f * w * m_orientation);
    m_orientation = (orientationDerivative * delta) * m_orientation;
    m_orientation = glm::normalize(m_orientation);
    m_linearMomentum = m_linearMomentum + delta * derivativeState.m_forces;
    m_angularMomentum = m_angularMomentum + delta * derivativeState.m_torques;
    m_time += delta;
  }
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

void RigidBody::setPosition(const glm::vec3 &position) {
  m_position = position;
}

glm::quat RigidBody::getOrientation() const {
  return m_orientation * getStandardOrientation();
}

void RigidBody::setOrientation(const glm::quat &orientation) {
  m_orientation = orientation;
}
std::optional<float>
RigidBody::getInterPenetration(const RigidBody &thatRigidBody) const {
  glm::mat4 thisTransform =
      glm::translate(glm::toMat4(getOrientation()), getPosition());
  glm::mat4 thatTransform = glm::translate(
      glm::toMat4(thatRigidBody.getOrientation()), thatRigidBody.getPosition());
  std::vector<std::shared_ptr<Collision>> planePointCollisions =
      m_hull.getPlanePointCollisions(thisTransform, thatRigidBody.m_hull,
                                     thatTransform);
  std::vector<std::shared_ptr<Collision>> edgeEdgeCollisions =
      m_hull.getEdgeEdgeCollisions(thisTransform, thatRigidBody.m_hull,
                                   thatTransform);
  std::optional<float> maxTimeOfContact;
  for (std::shared_ptr<Collision> &collision : planePointCollisions) {
    float timeOfContact = calculateTimeOfContact(thatRigidBody, collision);
    if (!maxTimeOfContact.has_value() || timeOfContact < maxTimeOfContact) {
      maxTimeOfContact = timeOfContact;
    }
  }
  for (std::shared_ptr<Collision> &collision : edgeEdgeCollisions) {
    float timeOfContact = calculateTimeOfContact(thatRigidBody, collision);
    if (!maxTimeOfContact.has_value() || timeOfContact < maxTimeOfContact) {
      maxTimeOfContact = timeOfContact;
    }
  }
  return maxTimeOfContact;
}
float RigidBody::calculateTimeOfContact(
    const RigidBody &thatRigidBody,
    const std::shared_ptr<Collision> collision) const {
  glm::vec3 point1 = collision->getPosition1();
  glm::vec3 point2 = collision->getPosition2();
  glm::vec3 normal = collision->getNormal();
  float thisMass = m_mass;
  glm::vec3 thisLinearVelocity = glm::vec3(0.0f);
  glm::vec3 thisAngularVelocity = glm::vec3(0.0f);
  glm::vec3 thisForce = glm::vec3(0.0f);
  glm::vec3 thisTorque = glm::vec3(0.0f);
  glm::vec3 point1Acceleration = glm::vec3(0.0f);
  if (thisMass != 0.0f) {
    thisLinearVelocity = m_linearMomentum / thisMass;
    thisAngularVelocity = m_IbodyInv * m_angularMomentum;
    thisForce = m_force;
    thisTorque = m_torque;
    point1Acceleration =
        glm::vec3(0.0f, -1.0f, 0.0f) * RigidBodySystem::c_gravity;
  }
  float thatMass = thatRigidBody.m_mass;
  glm::vec3 thatLinearVelocity = glm::vec3(0.0f);
  glm::vec3 thatAngularVelocity = glm::vec3(0.0f);
  glm::vec3 thatForce = glm::vec3(0.0f);
  glm::vec3 thatTorque = glm::vec3(0.0f);
  glm::vec3 point2Acceleration = glm::vec3(0.0f);
  if (thatMass != 0.0f) {
    thatLinearVelocity = thatRigidBody.m_linearMomentum / thatMass;
    thatAngularVelocity =
        thatRigidBody.m_IbodyInv * thatRigidBody.m_angularMomentum;
    thatForce = thatRigidBody.m_force;
    thatTorque = thatRigidBody.m_torque;
    point1Acceleration =
        glm::vec3(0.0f, -1.0f, 0.0f) * RigidBodySystem::c_gravity;
  }
  glm::vec3 point1Velocity =
      thisLinearVelocity + glm::cross(point1, thisAngularVelocity);
  glm::vec3 point2Velocity =
      thatLinearVelocity + glm::cross(point2, thatAngularVelocity);
  float relativeDistance = glm::dot(normal, point1 - point2);
  float relativeVelocity = glm::dot(normal, point1Velocity - point2Velocity);
  float relativeAcceleration =
      glm::dot(normal, point1Acceleration - point2Acceleration) -
      glm::dot(2.0f * (thatAngularVelocity), point1Velocity - point2Velocity);
  float squareRootPart = relativeVelocity * relativeVelocity -
                         2.0f * relativeAcceleration * relativeDistance;
  std::cout << "to square root: " << squareRootPart << std::endl;
  float root1 =
      (-relativeVelocity + glm::sqrt(squareRootPart)) / relativeAcceleration;
  float root2 =
      (-relativeVelocity - glm::sqrt(squareRootPart)) / relativeAcceleration;
  std::cout << "root 1: " << root1 << std::endl;
  std::cout << "root 2: " << root2 << std::endl;
  return root1 > root2 ? root1 : root2;
}
const glm::mat4 RigidBody::getTransform() const {

  const glm::quat orientation = getOrientation();
  const glm::mat4 translation = glm::translate(glm::mat4(1.0f), getPosition());
  return translation * glm::mat4_cast(orientation);
}

bool RigidBody::queryWitness(RigidBody &thatRigidBody) {
  std::shared_ptr<Witness> witness = m_hull.getWitness(
      getTransform(), thatRigidBody.m_hull, thatRigidBody.getTransform());
  if (witness.get() == nullptr) {
    return false;
  }
  m_witness = witness;
  return true;
}
const glm::quat RigidBody::getStandardOrientation() const {
  return glm::rotate(glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::radians(90.0f),
                     glm::vec3(1.0f, 0.0f, 0.0f));
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

  if (!m_isStopped) {
    for (RigidBody &rigidBody : m_rigidBodies) {
      rigidBody.addForcesAndTorques(glm::vec3(0.0f, 0.0f, 0.0f),
                                    glm::vec3(0.0f));
      rigidBody.eulerStep(delta);
      rigidBody.clearForcesAndTorques();
    }
    for (size_t i = 0; i < m_rigidBodies.size(); i++) {

      for (size_t j = i + 1; j < m_rigidBodies.size(); j++) {
        if (i != j) {
          bool witnesExists = m_rigidBodies[i].queryWitness(m_rigidBodies[j]);
          if (!witnesExists) {
            std::cout << "Collision: " << std::endl;
            stopSimulation();
          }
        }
      }
    }
  }
}

void RigidBodySystem::addRigidBody(const RigidBody &rigidBody) {
  m_rigidBodies.push_back(rigidBody);
}

Body RigidBodySystem::addMesh(const Engine::MeshNode &meshNode,
                              const float &mass,
                              const glm::vec3 &initialPosition,
                              const glm::quat &initialOrientation,
                              const glm::vec3 &initialVelocity,
                              const glm::vec3 &initialAngularVelocity) {
  std::vector<glm::vec3> vectorData;
  for (size_t i = 0; i < meshNode.meshData.position.size(); i += 3) {
    float x = meshNode.meshData.position.at(i);
    float y = meshNode.meshData.position.at(i + 1);
    float z = meshNode.meshData.position.at(i + 2);
    vectorData.push_back(glm::vec3(x, y, z));
  }
  ph::QuickHull qh(vectorData, meshNode.name);

  std::vector<glm::vec3> vectorHull;
  const std::vector<float> positionBuffer = meshNode.meshData.position;
  for (size_t i = 0; i < positionBuffer.size(); i += 3) {
    vectorHull.push_back(glm::vec3(positionBuffer.at(i),
                                   positionBuffer.at(i + 1),
                                   positionBuffer.at(i + 2)));
  }

  const glm::vec3 &centerOfMass = glm::vec3(0.0f);

  glm::mat3 inertialTensor = glm::mat3(1.0f);

  if (mass != 0.0f) {
    // inertialTensor = getInertiaTensor(vertexBuffer, indexBuffer, false);
  }
  float meshMass = mass;
  if (meshNode.mass != 0.0f) {
    meshMass = meshNode.mass;
  }
  m_rigidBodies.push_back(RigidBody(qh, meshMass, inertialTensor,
                                    meshNode.position, meshNode.orientation,
                                    initialVelocity, initialAngularVelocity));

  return Body(m_rigidBodies.size() - 1, *this);
}
const glm::vec3 RigidBodySystem::getPosition(const uint32_t index) {
  return m_rigidBodies[index].getPosition();
}
const glm::mat4 RigidBodySystem::getTransform(const uint32_t index) {
  return m_rigidBodies[index].getTransform();
}

void RigidBodySystem::stopSimulation() { m_isStopped = true; }
void RigidBodySystem::resumeSimulation() { m_isStopped = false; }
Body::Body(const uint32_t index, RigidBodySystem &system)
    : m_index(index), m_system(system) {}
glm::vec3 Body::getPosition() const { return m_system.getPosition(m_index); }
glm::mat4 Body::getTransform() const { return m_system.getTransform(m_index); }
} // namespace ph
