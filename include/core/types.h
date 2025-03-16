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

using Quaterniond = Eigen::Quaterniond;
using AngleAxisd = Eigen::AngleAxisd;

inline Matrix4d idk(Vector3d w) {
  Eigen::Matrix4d W;
  W <<     0, -w.x(), -w.y(), -w.z(),
       w.x(),      0,  w.z(), -w.y(),
       w.y(), -w.z(),      0,  w.x(),
       w.z(),  w.y(), -w.x(),      0;
  return W;
}

// Multiplies two Vector4d quaternions [x, y, z, w]
inline Vector4d quaternionMultiply(const Vector4d &a, const Vector4d &b) {
  return Vector4d(
    a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(),
    a.w() * b.y() - a.x() * b.z() + a.y() * b.w() + a.z() * b.x(),
    a.w() * b.z() + a.x() * b.y() - a.y() * b.x() + a.z() * b.w(),
    a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z()
  );
}

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

inline void toggle(bool &value) {
  value = !value;
}

} // namespace Neutron

#endif // TYPES_H