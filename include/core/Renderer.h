#ifndef RENDERER_H
#define RENDERER_H

#include <memory>

#include "Camera.h"
#include "utils/Logger.h"


class Renderer {
public:
  Renderer();
  ~Renderer();

  bool initialize();
  void clearScreen() ;
  void render(const Camera& camera);

};


#endif // RENDERER_H
