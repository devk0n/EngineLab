#include <glad/glad.h>

#include "core/WindowManager.h"
#include "utils/Logger.h"

WindowManager::WindowManager(std::string title)
  : m_window(nullptr, glfwDestroyWindow),
    m_width(0),
    m_height(0),
    m_title(std::move(title)) {}

bool WindowManager::initialize() {
  if (!glfwInit()) {
    LOG_ERROR("Failed to initialize GLFW");
    return false;
  }

  // Get primary monitor and its video mode
  GLFWmonitor* primaryMonitor = glfwGetPrimaryMonitor();
  const GLFWvidmode* videoMode = glfwGetVideoMode(primaryMonitor);

  // Use 80% of monitor's dimensions as window size
  m_width = static_cast<int>(videoMode->width * 0.8);
  m_height = static_cast<int>(videoMode->height * 0.8);

  // Ensure minimum size for small monitors
  m_width = std::max(m_width, 800);
  m_height = std::max(m_height, 600);

  // Set OpenGL context hints
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

  // Create window
  GLFWwindow* rawWindow = glfwCreateWindow(m_width, m_height, m_title.c_str(), nullptr, nullptr);
  if (!rawWindow) {
    LOG_ERROR("Failed to create GLFW window");
    glfwTerminate();
    return false;
  }

  // Center window on screen
  int xPos = (videoMode->width - m_width) / 2;
  int yPos = (videoMode->height - m_height) / 2;
  glfwSetWindowPos(rawWindow, xPos, yPos);

  m_window.reset(rawWindow);
  glfwMakeContextCurrent(m_window.get());

  if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
    LOG_ERROR("Failed to initialize GLAD");
    return false;
  }

  LOG_INFO("Window initialized!");
  LOG_DEBUG("Monitor size: " + std::to_string(videoMode->width) + "x" + std::to_string(videoMode->height));
  LOG_DEBUG("Monitor refresh rate: " + std::to_string(videoMode->refreshRate) + "Hz");
  LOG_DEBUG("Monitor aspect ratio: " + std::to_string(videoMode->width / static_cast<float>(videoMode->height)));
  LOG_DEBUG("Window size: " + std::to_string(m_width) + "x" + std::to_string(m_height));
  LOG_DEBUG("Window aspect ratio: " + std::to_string(m_width / static_cast<float>(m_height)));
  LOG_DEBUG("OpenGL version: " + std::string(reinterpret_cast<const char*>(glGetString(GL_VERSION))));
  LOG_DEBUG("OpenGL vendor: " + std::string(reinterpret_cast<const char*>(glGetString(GL_VENDOR))));
  LOG_DEBUG("OpenGL renderer: " + std::string(reinterpret_cast<const char*>(glGetString(GL_RENDERER))));
  LOG_DEBUG("OpenGL shading language version: " + std::string(reinterpret_cast<const char*>(glGetString(GL_SHADING_LANGUAGE_VERSION))));

  return true;
}

void WindowManager::pollEvents() { glfwPollEvents(); }
void WindowManager::swapBuffers() const { glfwSwapBuffers(m_window.get()); }
bool WindowManager::shouldClose() const { return glfwWindowShouldClose(m_window.get()); }
void WindowManager::close() const { glfwSetWindowShouldClose(m_window.get(), true); }
