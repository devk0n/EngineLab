#include <cstdlib>

#include "core/Application.h"
#include "utils/Logger.h"

int main() {
  // Initialize logger
  if (!Logger::initialize("log.txt")) {
    return EXIT_FAILURE;
  }
  Logger::setLogLevel(Logger::Level::Debug);
  LOG_INFO("Logger initialized!");

  // Run application
  Application app;
  if (!app.initialize()) {
    return EXIT_FAILURE;
  }
  app.run();

  return EXIT_SUCCESS;
}