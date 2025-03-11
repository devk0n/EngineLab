
void DynamicSystem::solveVelocityConstraints(
    const double epsilon = 1e-6,
    const int maxIterations = 10,
    const double alpha = 0.5,         // Relaxation parameter
    const double lambda = 1e-6,       // Use damped least squares
    const double maxCorrection = 0.1) // Maximum velocity correction per iteration
{
  if (m_particles.empty()) return;

  std::vector<Particle *> particles;
  for (const auto &[id, particle]: m_particles) {
    particles.push_back(particle.get());
  }

  for (int iter = 0; iter < maxIterations; ++iter) {
    // Build Jacobian
    MatrixXd jacobian;
    VectorXd dummy;
    m_constraintSolver.buildJacobian(particles, jacobian, dummy);

    // Compute current velocity constraint violation
    VectorXd qdot(particles.size() * 3);
    for (int i = 0; i < particles.size(); ++i) {
      qdot.segment<3>(i * 3) = particles[i]->getVelocity();
    }

    VectorXd cdot = jacobian * qdot;

    // Check if velocity constraints are satisfied
    if (cdot.norm() < epsilon) {
      break;
    }

    MatrixXd JTJ = jacobian.transpose() * jacobian;
    MatrixXd H = JTJ + lambda * MatrixXd::Identity(JTJ.rows(), JTJ.cols());
    VectorXd dv = H.ldlt().solve(jacobian.transpose() * (-cdot));

    // Apply scaled corrections to velocities
    int i = 0;
    for (auto &particle: particles) {
      if (!particle->isFixed()) {
        Vector3d correction = alpha * dv.segment<3>(i * 3);

        // Limit correction magnitude
        if (correction.norm() > maxCorrection) {
          correction *= maxCorrection / correction.norm();
        }
        particle->setVelocity(particle->getVelocity() + correction);
      }
      i++;
    }
  }
}
