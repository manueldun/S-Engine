#include "Engine.h"
#include "tiny_gltf.h"
#include <cstdlib>
#include <glm/common.hpp>
#include <iostream>
#include <string>
int main(int argc, char **argv) {

  try {
    Engine::Engine engine;
    while(!engine.shouldExit()){
      engine.loop();
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
