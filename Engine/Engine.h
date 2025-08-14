#pragma once

#include "./common/Interfaces.h"
#include "Physics.h"
#include "Renderer.h"
#include <string>
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
class Engine : public Observer {
public:
  Engine();
  void loadScene(const std::string &path);
  void loop();
  bool shouldExit();
  void onNotify(const Event &event,
                const std::variant<std::string> &data) override;

private:
  std::vector<Entity> entities;
  Renderer::Renderer renderer;
  ph::RigidBodySystem physicsSystem;
};
} // namespace Engine
