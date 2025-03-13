#include "Solver.h"

#include "Utils/Logger.h"

namespace Neutron {

void Solver::solveSystem(
    const VectorXd &M,
    const VectorXd &forces,
    VectorXd &accelerations) const {

  accelerations = M.asDiagonal().inverse() * (forces);

}

}


