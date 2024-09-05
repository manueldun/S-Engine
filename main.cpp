#include "Renderer.h"

#include "glm/ext/matrix_transform.hpp"
#include "glm/fwd.hpp"
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
    RenderObject coin = app.loadGLTF("/home/manuel/Documents/assets/"
                                     "KayKit_DungeonRemastered_1.1_FREE/Assets/"
                                     "gltf/coin.gltf");
    while (!app.shouldExit()) {

      static auto startTime = std::chrono::high_resolution_clock::now();

      auto currentTime = std::chrono::high_resolution_clock::now();
      float time = std::chrono::duration<float, std::chrono::seconds::period>(
                       currentTime - startTime)
                       .count();
      glm::mat4 coinMat =
          glm::rotate(glm::mat4(1.0f), time, glm::vec3(0.0, 0.0, 1.0));
      coin.setMatrix(coinMat);
      app.loop();
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
