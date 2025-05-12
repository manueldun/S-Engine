#include "Physics.h"
#include "Renderer.h"
#include "glm/ext/matrix_transform.hpp"
#include "glm/fwd.hpp"
#include "imgui_impl_glfw.h"
#include <chrono>
#include <glm/common.hpp>
#include <iostream>
int main(int argc, char **argv) {
  try {
    Renderer app;
    /*RenderObject table =*/
    /*    app.loadGLTF("/home/manuel/Documents/assets/"*/
    /*                 "KayKit_DungeonRemastered_1.1_FREE/Assets/"*/
    /*                 "gltf/table_small.gltf");*/

    RenderObject coin = app.loadGLTF("/home/manuel/3d-assets/"
                                     "KayKit_DungeonRemastered_1.1_FREE/Assets/"
                                     "gltf/coin.gltf");
    RenderObject cube = app.loadGLTF("/home/manuel/3d-assets/test_assets/cube.gltf");
    Physics::ParticleSystem particleSystem;

    Physics::Plane floor(glm::vec3(0.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    particleSystem.addPlane(floor);

    Physics::Particle coinParticle(glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f),
                                   glm::vec3(0.0f), 1.0f);

    particleSystem.addParticle(coinParticle);

    auto startTime = std::chrono::high_resolution_clock::now();
    auto currentTime = std::chrono::high_resolution_clock::now();
    while (!app.shouldExit()) {

      currentTime = std::chrono::high_resolution_clock::now();
      float deltaTime =
          std::chrono::duration<float, std::chrono::seconds::period>(
              currentTime - startTime)
              .count();

      glm::mat4 coinMat = glm::translate(glm::mat4(1.0f),
                                         particleSystem.getParticlePosition(0));
      coin.setMatrix(coinMat, 0);
      particleSystem.eulerStep(deltaTime);

      app.draw(&coin);

      app.endFrame();
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
