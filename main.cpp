#include "Renderer.h"
#include <iostream>
int main(int argc, char **argv) {
  Renderer app;
  try {
    app.init();
    app.loadGLTF("/media/manuel/Disco Auxiliar/linux mint documents "
                 "9-4-2024/assets/KayKit_DungeonRemastered_1.1_FREE/Assets/"
                 "gltf/table_small.gltf");
    while (!app.shouldExit()) {
      app.loop();
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
