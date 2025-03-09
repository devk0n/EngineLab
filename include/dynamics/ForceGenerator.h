#ifndef FORCE_GENERATOR_H
#define FORCE_GENERATOR_H

class ForceGenerator {
public:
  virtual ~ForceGenerator() = default;
  virtual void apply(double dt) = 0;
};

#endif // FORCE_GENERATOR_H
