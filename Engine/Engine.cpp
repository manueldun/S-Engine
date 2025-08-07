#include "Engine.h"
#include "Mesh.h"
#include "Physics.h"
#include "Renderer.h"
#include "utils.h"
#include <chrono>
namespace Engine {
Entity::Entity(const std::shared_ptr<Renderer::Drawing> &drawing,
               const ph::Body &body)
    : drawing(drawing), body(body) {}

void Entity::update() { drawing->setTranform(body.getTransform()); }
void Engine::loadScene(const std::string &path) {

  const tinygltf::Model &sceneModel = util::loadGltfFile(path);
  const Scene scene(sceneModel);
  for (const std::shared_ptr<MeshNode> &meshNode : scene.getMeshNodes()) {
    auto drawing = renderer.loadModel(*meshNode);
    auto body = physicsSystem.addMesh(*meshNode);
    entities.push_back(Entity(drawing, body));
  }

}
void Engine::loop() {

  static auto startTime = std::chrono::system_clock::now();
  static auto currentTime = std::chrono::system_clock::now();
  currentTime = std::chrono::system_clock::now();
  static auto deltaTime = currentTime - startTime;
  deltaTime = currentTime - startTime;
  while (deltaTime > std::chrono::milliseconds(16)) {
    physicsSystem.eulerStep(std::chrono::milliseconds(16).count() / 1000.0f);
    startTime += deltaTime;
    deltaTime -= std::chrono::milliseconds(16);
  }
  for (Entity &entity : entities) {
    entity.update();
    renderer.draw(entity.drawing);
  }

  renderer.endFrame();
}
bool Engine::shouldExit() { return renderer.shouldExit(); }
} // namespace Engine
