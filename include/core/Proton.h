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

} // Proton

#endif // PROTON_H
