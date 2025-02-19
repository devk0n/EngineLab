#ifndef RENDERER_H
#define RENDERER_H

#include <GLFW/glfw3.h>

#include "utils/Logger.h"


class Renderer {
public:
  Renderer();
  ~Renderer();

  bool initialize();
  void clearScreen() ;
  void render();

};


#endif // RENDERER_H
