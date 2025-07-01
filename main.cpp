#include "Physics.h"
#include "Renderer.h"
#include "glm/ext/matrix_transform.hpp"
#include "glm/fwd.hpp"
#include "tiny_gltf.h"
#include "utils.h"
#include <chrono>
#include <cstdlib>
#include <glm/common.hpp>
#include <iostream>
#include <string>
int main(int argc, char **argv) {
  try {
    Renderer app;
    /*RenderObject table =*/
    /*    app.loadGLTF("/home/manuel/Documents/assets/"*/
    /*                 "KayKit_DungeonRemastered_1.1_FREE/Assets/"*/ /*                 "gltf/table_small.gltf");*/

    const tinygltf::Model &modelCoin =
        loadGltfFile("/home/manuel/3d-assets/test_assets/cube.gltf");
    RenderObject coin = app.loadModel(modelCoin);

    Physics::RigidBodySystem system;
    Physics::RigidBody& coinBody = system.addMesh(modelCoin, 1.0f);

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
