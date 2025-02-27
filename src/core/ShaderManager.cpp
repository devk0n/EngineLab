#include "core/ShaderManager.h"
#include <fstream>
#include <sstream>

#include "utils/Logger.h"

ShaderManager::~ShaderManager() { cleanup(); }

bool ShaderManager::loadShader(const std::string &name,
                               const std::string &vertexPath,
                               const std::string &fragmentPath) {
  if (m_shaders.contains(name)) {
    LOG_WARN("Shader '", name, "' already loaded.");
    return true;
  }

  std::string vertexSource = readFile(vertexPath);
  std::string fragmentSource = readFile(fragmentPath);
  if (vertexSource.empty() || fragmentSource.empty()) {
    LOG_ERROR("Failed to read shader files for '", name, "'");
    return false;
  }

  unsigned int vertexShader = compileShader(GL_VERTEX_SHADER, vertexSource);
  unsigned int fragmentShader =
      compileShader(GL_FRAGMENT_SHADER, fragmentSource);
  if (vertexShader == 0 || fragmentShader == 0) {
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    return false;
  }

  unsigned int program = glCreateProgram();
  glAttachShader(program, vertexShader);
  glAttachShader(program, fragmentShader);

  if (!linkProgram(program, vertexShader, fragmentShader)) {
    glDeleteProgram(program);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    return false;
  }

  glDeleteShader(vertexShader);
  glDeleteShader(fragmentShader);

  m_shaders[name] = program;
  LOG_INFO("Shader '", name, "' loaded successfully");
  return true;
}

unsigned int ShaderManager::getShader(const std::string &name) const {
  if (auto it = m_shaders.find(name); it != m_shaders.end())
    return it->second;
  LOG_ERROR("Shader '", name, "' not found");
  return 0;
}

void ShaderManager::cleanup() {
  for (auto &pair: m_shaders)
    glDeleteProgram(pair.second);
  m_shaders.clear();
}

std::string ShaderManager::readFile(const std::string &filePath) const {
  std::ifstream file(filePath);
  if (!file.is_open()) {
    LOG_ERROR("Failed to open file: ", filePath);
    return "";
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

unsigned int ShaderManager::compileShader(GLenum type,
                                          const std::string &source) const {
  unsigned int shader = glCreateShader(type);
  const char *src = source.c_str();
  glShaderSource(shader, 1, &src, nullptr);
  glCompileShader(shader);

  int success;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
  if (!success) {
    char infoLog[512];
    glGetShaderInfoLog(shader, 512, nullptr, infoLog);
    LOG_ERROR("Shader compilation failed: ", infoLog);
    return 0;
  }
  return shader;
}

bool ShaderManager::linkProgram(unsigned int program, unsigned int vertexShader,
                                unsigned int fragmentShader) const {
  glLinkProgram(program);

  int success;
  glGetProgramiv(program, GL_LINK_STATUS, &success);
  if (!success) {
    char infoLog[512];
    glGetProgramInfoLog(program, 512, nullptr, infoLog);
    LOG_ERROR("Shader program linking failed: ", infoLog);
    return false;
  }
  return true;
}
