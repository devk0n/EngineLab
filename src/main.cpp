#include "core/Application.h"

int main() {

  // Initialize logger
  if (!Logger::initialize("log.txt")) {
    return false;
  }
  Logger::setLogLevel(Logger::Level::Debug);
  LOG_ERROR("Logger initialized.");

  Application app;
  if (!app.initialize()) {
    LOG_ERROR("Failed to initialize application.");
    return 1;
  }

  app.run();

  LOG_DEBUG("Application terminated.");
}
