#include <glad/glad.h>

#include "core/WindowManager.h"
#include "utils/Logger.h"

WindowManager::WindowManager(const int width, const int height, std::string title)
  : m_window(nullptr, glfwDestroyWindow),
    m_width(width),
    m_height(height),
    m_title(std::move(title)) {
}

WindowManager::~WindowManager() {
  LOG_DEBUG("Destroying GLFW window.");
}

bool WindowManager::initialize() {
  glfwSetErrorCallback(GLFWErrorCallback);

  if (!glfwInit()) {
    LOG_ERROR("Failed to initialize GLFW!");
    return false;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

  GLFWwindow *rawWindow = glfwCreateWindow(m_width, m_height, m_title.c_str(), nullptr, nullptr);
  if (!rawWindow) {
    LOG_ERROR("Failed to create GLFW window!");
    return false;
  }

  m_window.reset(rawWindow);
  glfwMakeContextCurrent(m_window.get());

  if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
    LOG_ERROR("Failed to initialize GLAD!");
    return false;
  }

  setGLFWCallbacks();
  LOG_INFO("WindowManager initialized successfully.");
  return true;
}

void WindowManager::setGLFWCallbacks() {
  LOG_DEBUG("Setting up GLFW callbacks...");

  // Example: Set up mouse movement callback
  glfwSetCursorPosCallback(m_window.get(), [](GLFWwindow* window, double xpos, double ypos) {
      // Handle mouse movement
  });

  // Example: Set up keyboard input callback
  glfwSetKeyCallback(m_window.get(), [](GLFWwindow* window, int key, int scancode, int action, int mods) {
      // Handle key input
  });
}

void WindowManager::pollEvents() const { glfwPollEvents(); }
void WindowManager::swapBuffers() const { glfwSwapBuffers(m_window.get()); }
bool WindowManager::shouldClose() const { return glfwWindowShouldClose(m_window.get()); }
void WindowManager::close() { glfwSetWindowShouldClose(m_window.get(), true); }

void WindowManager::GLFWErrorCallback(int error, const char *description) {
  LOG_ERROR("GLFW Error ({}): {}", error, description);
}
