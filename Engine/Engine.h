#pragma once

#include <string>
#include "Physics.h"
#include "Renderer.h"
namespace Engine {
class Entity {
public:
  Entity(const std::shared_ptr<Renderer::Drawing> &drawing,
         const ph::Body &body);
  void update();
  const std::shared_ptr<Renderer::Drawing> drawing;
  const ph::Body body;
private:
};
class Engine {
public:
  void loadScene(const std::string &path);
  void loop();
  bool shouldExit();

private:
  std::vector<Entity> entities;
  Renderer::Renderer renderer;
  ph::RigidBodySystem physicsSystem;
};
} // namespace Engine
