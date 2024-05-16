#include "Renderer.h"
#include <iostream>
int main(int argc, char **argv) {
  Renderer app;
  try {
    app.run();
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
