#include "Physics.h"
#include "Renderer.h"
#include "glm/ext/matrix_transform.hpp"
#include "glm/fwd.hpp"
#include "utils.h"
#include <chrono>
#include <cstdlib>
#include <glm/common.hpp>
#include <iostream>
#include <string>
#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "tiny_gltf.h"
int main(int argc, char **argv) {
  try {
    Renderer app;
    /*RenderObject table =*/
    /*    app.loadGLTF("/home/manuel/Documents/assets/"*/
    /*                 "KayKit_DungeonRemastered_1.1_FREE/Assets/"*/
    /*                 "gltf/table_small.gltf");*/

    const tinygltf::Model model =
        loadGltfFile("/home/manuel/3d-assets/KayKit_DungeonRemastered_1.1_FREE/"
                     "Assets/gltf/coin.gltf");
    RenderObject coin = app.loadModel(model);
    RenderObject cube =
        app.loadModel(model);
    Physics::RigidBodySystem system;
    Physics::RigidBody coinBody(
        1.0f, glm::mat3(1.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::mat3(1.0f),
        glm::vec3(0.0f, 0.0f, 3.0f), glm::vec3(0.0f, 3.0f, 0.0f));
    system.addRigidBody(&coinBody);

    auto startTime = std::chrono::system_clock::now();
    auto currentTime = std::chrono::system_clock::now();
    std::chrono::duration deltaTime = currentTime - startTime;
    while (!app.shouldExit()) {

      currentTime = std::chrono::system_clock::now();
      glm::mat4 coinMat =
          glm::translate(glm::mat4(1.0f), coinBody.getPosition());
      coinMat = coinMat * glm::mat4(coinBody.getOrientation());
      deltaTime = currentTime - startTime;
      coin.setMatrix(coinMat, 0);
      while (deltaTime > std::chrono::milliseconds(16)) {
        system.eulerStep(std::chrono::milliseconds(16).count() / 1000.0f);
        startTime += deltaTime;
        deltaTime -= std::chrono::milliseconds(16);
      }

      app.draw(&coin);
      std::string coinHeight = std::to_string(deltaTime.count());
      app.drawLabel(&coinHeight);

      app.endFrame();
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
