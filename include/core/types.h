// core/types.h
#ifndef TYPES_H
#define TYPES_H

#include <cstdint>
#include <Eigen/Dense>

namespace Neutron {

using UniqueID = std::uint32_t;

// Common vector/matrix types
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;
using Quaterniond = Eigen::Quaterniond;

inline void skew(const Vector3d&) {

}

} // namespace Neutron

#endif // TYPES_H