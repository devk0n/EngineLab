#ifndef PROTON_H
#define PROTON_H

#include <Eigen/Dense>
#include "Logger.h"

namespace Proton {

using UniqueID = std::uint16_t;

using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using VectorXd = Eigen::VectorXd;

using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using MatrixXd = Eigen::MatrixXd;

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

inline Eigen::Matrix4d omegaMatrix(const Eigen::Vector3d& w) {
  Eigen::Matrix4d Omega;
  Omega <<  0,    -w.x(), -w.y(), -w.z(),
            w.x(),  0,     w.z(), -w.y(),
            w.y(), -w.z(), 0,     w.x(),
            w.z(),  w.y(), -w.x(), 0;
  return Omega;
}

inline Eigen::Vector4d applySmallRotationQuaternion(const Eigen::Vector4d& q, const Eigen::Vector3d& deltaTheta) {
  Eigen::Matrix4d Omega = omegaMatrix(deltaTheta);
  Eigen::Vector4d dq = 0.5 * Omega * q;
  Eigen::Vector4d q_new = q + dq;
  q_new.normalize();
  return q_new;
}


} // Proton

#endif // PROTON_H
