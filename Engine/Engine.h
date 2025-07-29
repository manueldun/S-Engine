#pragma once

#include <string>
#include "Mesh.h"
#include "Physics.h"
#include "Renderer.h"
namespace Engine {
class Entity {
public:
  Entity(const std::shared_ptr<Renderer::Drawing> &drawing,
         const Physics::Body &body);
  void update();
  const std::shared_ptr<Renderer::Drawing> drawing;
  const Physics::Body body;
private:
};
class Engine {
public:
  void loadScene(const std::string &path);
  void loop();
  bool shouldExit();

private:
  void addEntities(const Scene &scene);
  std::vector<Entity> entities;
  Renderer::Renderer renderer;
  Physics::RigidBodySystem physicsSystem;
};
} // namespace Engine
