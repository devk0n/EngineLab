// core/types.h
#ifndef TYPES_H
#define TYPES_H

#include <cstdint>
#include <Eigen/Dense>
#include "utils/Logger.h"

namespace Neutron {

using UniqueID = std::uint16_t;

// Common vector/matrix types
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;

using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;

using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;


inline Eigen::Matrix<double, 3, 4> GMatrix(Vector4d e) {
  Eigen::Matrix<double, 3, 4> G;
  G << -e.x(),  e.w(), -e.z(),  e.y(),
       -e.y(),  e.z(),  e.w(), -e.x(),
       -e.z(), -e.y(),  e.x(),  e.w();
  return G;
}

inline Eigen::Matrix<double, 3, 4> LMatrix(Vector4d e) {
  Eigen::Matrix<double, 3, 4> L;
  L << -e.x(),  e.w(),  e.z(), -e.y(),
       -e.y(), -e.z(),  e.w(),  e.x(),
       -e.z(),  e.y(), -e.x(),  e.w();
  return L;
}

inline Matrix3d skew(Vector3d v) {
  Matrix3d skew;
  skew <<  0,    -v.z(),  v.y(),
           v.z(),  0,    -v.x(),
          -v.y(),  v.x(),  0;
  return skew;
}

} // namespace Neutron

#endif // TYPES_H