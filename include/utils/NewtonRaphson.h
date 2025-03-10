#ifndef NEWTON_RAPHSON_H
#define NEWTON_RAPHSON_H

#include <limits>
#include <stdexcept>
#include <Eigen/Dense>

template<typename Scalar, typename Function, typename Jacobian>
class NewtonRaphson {
public:
  // Constructor
  NewtonRaphson() = default;

  // Main solver method for vector-valued functions
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> solve(
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& initialGuess,
      Function function,
      Jacobian jacobian,
      Scalar convergenceTolerance,
      int maxIterations) const;

  // Optional: Add additional utility methods if needed
};

// Implementation of the solve method for vector-valued functions
template<typename Scalar, typename Function, typename Jacobian>
Eigen::Matrix<Scalar, Eigen::Dynamic, 1> NewtonRaphson<Scalar, Function, Jacobian>::solve(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& initialGuess,
    Function function,
    Jacobian jacobian,
    Scalar convergenceTolerance,
    const int maxIterations) const {

  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> currentEstimate = initialGuess;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> functionValue = function(currentEstimate);
  const Scalar minDerivative = std::numeric_limits<Scalar>::epsilon();

  for (int iteration = 0; iteration < maxIterations; ++iteration) {
    // Check for convergence
    if (functionValue.norm() < convergenceTolerance)
      return currentEstimate;

    // Compute the Jacobian
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> jacobianValue = jacobian(currentEstimate);

    // Check if the Jacobian is invertible
    // if (jacobianValue.determinant() < minDerivative)
    //   throw std::runtime_error("Jacobian is singular or nearly singular");

    // Solve for the step: J * delta = -f
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> step = jacobianValue.colPivHouseholderQr().solve(-functionValue);

    // Update the estimate
    currentEstimate += step;

    // Update the function value
    functionValue = function(currentEstimate);

    // Check for convergence based on the step size
    if (step.norm() < convergenceTolerance)
      return currentEstimate;
  }

  // If we reach here, the method did not converge
  throw std::runtime_error("Newton-Raphson failed to converge within the specified iterations");
}

#endif // NEWTON_RAPHSON_H