#ifndef FORCEGENERATOR_H
#define FORCEGENERATOR_H

#include "Particle.h"

class ForceGenerator {
public:
  virtual ~ForceGenerator() = default;
  virtual void apply(double dt) = 0;
};



#endif // FORCEGENERATOR_H
