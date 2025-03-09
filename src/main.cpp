#include "core/Application.h"
#include "utils/Logger.h"

int main() {

  // Initialize logger
  if (!Logger::initialize("log.txt")) {
    return false;
  }
  Logger::ConsoleConfig config;
  config.showTimestamps = false; // Disable timestamps
  config.showFileNames = false;  // Disable file names
  config.showLevel = false;      // Disable level
  config.enabled = false;        // Disable console logger
  Logger::setConsoleConfig(config);
  Logger::setLogLevel(Logger::Level::Info);
  LOG_INFO("Logger initialized.");

  Application app;
  if (!app.initialize()) {
    LOG_ERROR("Failed to initialize application.");
    return 1;
  }

  app.run();

  LOG_DEBUG("Application terminated.");
}
