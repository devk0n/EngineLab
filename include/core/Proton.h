#ifndef PROTON_H
#define PROTON_H

#include "pch.h"

namespace Proton {

using UniqueID = std::uint16_t;

using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using VectorXd = Eigen::VectorXd;

using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using MatrixXd = Eigen::MatrixXd;

using MatrixG = Eigen::Matrix<double, 3, 4>;
using MatrixL = Eigen::Matrix<double, 3, 4>;

using Quaterniond = Eigen::Quaterniond;

inline bool initializeLogger() {
  // Initialize logger
  if (!Logger::initialize("log.txt")) { return false; }
  Logger::setLogLevel(Logger::Level::Debug);
  Logger::ConsoleConfig config;
  config.showTimestamps = true; // Disable timestamps
  config.showFileNames = true;  // Disable file names
  config.showLevel = true;      // Disable level
  Logger::setConsoleConfig(config);
  LOG_INFO("Logger initialized.");
  return true;
}

inline Matrix4d omegaMatrix(const Vector3d& w) {
  Matrix4d Omega;
  Omega <<  0,    -w.x(), -w.y(), -w.z(),
            w.x(),  0,     w.z(), -w.y(),
            w.y(), -w.z(), 0,     w.x(),
            w.z(),  w.y(), -w.x(), 0;
  return Omega;
}

inline Vector4d applySmallRotationQuaternion(const Vector4d& q, const Vector3d& deltaTheta) {
  Matrix4d Omega = omegaMatrix(deltaTheta);
  Vector4d dq = 0.5 * Omega * q;
  Vector4d q_new = q + dq;
  q_new.normalize();
  return q_new;
}

inline Vector4d integrateQuaternion(const Vector4d& q, const Vector3d& omega, double dt) {
  Matrix4d Omega = omegaMatrix(omega);
  Vector4d dq = 0.5 * Omega * q;
  Vector4d q_new = q + dt * dq;
  q_new.normalize();
  return q_new;
}

inline Matrix3d skew(const Vector3d& v) {
  Matrix3d S;
  S <<     0,   -v.z(),  v.y(),
         v.z(),     0,  -v.x(),
        -v.y(),  v.x(),     0;
  return S;
}

inline MatrixG matrixG(Vector4d e) {
  MatrixG G;
  G << -e[1],  e[0], -e[3],  e[2],
       -e[2],  e[3],  e[0], -e[1],
       -e[3], -e[2],  e[1],  e[0];
  return G;
}

inline MatrixL matrixL(Vector4d e) {
  MatrixG L;
  L << -e[1],  e[0],  e[3], -e[2],
       -e[2], -e[3],  e[0],  e[1],
       -e[3],  e[2], -e[1],  e[0];
  return L;
}

} // Proton

#endif // PROTON_H
