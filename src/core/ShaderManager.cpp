#include "core/ShaderManager.h"
#include <fstream>
#include <sstream>

#include "utils/Logger.h"

ShaderManager::~ShaderManager() { cleanup(); }

bool ShaderManager::loadShader(const std::string &name,
                               const std::string &vertexPath,
                               const std::string &fragmentPath,
                               const std::string &geometryPath) {
  std::lock_guard<std::mutex> lock(m_mutex);

  // Read shader files
  std::string vertexSource = readFile(vertexPath);
  std::string fragmentSource = readFile(fragmentPath);
  std::string geometrySource;
  if (!geometryPath.empty()) {
    geometrySource = readFile(geometryPath);
  }

  // Compile shaders
  unsigned int vertexShader = compileShader(GL_VERTEX_SHADER, vertexSource);
  unsigned int fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentSource);
  unsigned int geometryShader = 0;
  if (!geometryPath.empty()) {
    geometryShader = compileShader(GL_GEOMETRY_SHADER, geometrySource);
  }

  // Link program
  unsigned int program = glCreateProgram();
  if (!linkProgram(program, vertexShader, fragmentShader, geometryShader)) {
    return false;
  }

  // Store program
  m_shaders[name] = program;

  // Clean up shaders
  glDeleteShader(vertexShader);
  glDeleteShader(fragmentShader);
  if (geometryShader != 0) {
    glDeleteShader(geometryShader);
  }

  return true;
}

bool ShaderManager::loadComputeShader(const std::string &name,
                                      const std::string &computePath) {
  std::lock_guard<std::mutex> lock(m_mutex);

  // Read compute shader file
  std::string computeSource = readFile(computePath);

  // Compile compute shader
  unsigned int computeShader = compileShader(GL_COMPUTE_SHADER, computeSource);
  if (computeShader == 0) {
    return false;
  }

  // Link program
  unsigned int program = glCreateProgram();
  glAttachShader(program, computeShader);
  glLinkProgram(program);

  if (!checkLinkErrors(program)) {
    glDeleteShader(computeShader);
    glDeleteProgram(program);
    return false;
  }

  // Store program
  m_shaders[name] = program;

  // Clean up shader
  glDeleteShader(computeShader);

  return true;
}

unsigned int ShaderManager::getShader(const std::string &name) const {
  std::lock_guard<std::mutex> lock(m_mutex);
  if (auto it = m_shaders.find(name); it != m_shaders.end()) {
    return it->second;
  }
  LOG_ERROR("Shader '", name, "' not found");
  return 0;
}

void ShaderManager::cleanup() {
  std::lock_guard<std::mutex> lock(m_mutex);
  for (auto &pair : m_shaders) {
    glDeleteProgram(pair.second);
  }
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

unsigned int ShaderManager::compileShader(GLenum type, const std::string &source) const {
  unsigned int shader = glCreateShader(type);
  const char *src = source.c_str();
  glShaderSource(shader, 1, &src, nullptr);
  glCompileShader(shader);

  if (!checkCompileErrors(shader, (type == GL_VERTEX_SHADER) ? "VERTEX" :
                             (type == GL_FRAGMENT_SHADER) ? "FRAGMENT" :
                             (type == GL_GEOMETRY_SHADER) ? "GEOMETRY" : "COMPUTE")) {
    glDeleteShader(shader);
    return 0;
                             }

  return shader;
}

bool ShaderManager::linkProgram(unsigned int program,
                                unsigned int vertexShader,
                                unsigned int fragmentShader,
                                unsigned int geometryShader) const {
  glAttachShader(program, vertexShader);
  glAttachShader(program, fragmentShader);
  if (geometryShader != 0) {
    glAttachShader(program, geometryShader);
  }
  glLinkProgram(program);

  return checkLinkErrors(program);
}

bool ShaderManager::checkCompileErrors(unsigned int shader, const std::string &type) const {
  int success;
  char infoLog[1024];
  glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(shader, 1024, nullptr, infoLog);
    LOG_ERROR("Shader compilation error (", type, "): ", infoLog);
    return false;
  }
  return true;
}

void ShaderManager::setUniform(const std::string &shaderName,
                               const std::string &uniformName,
                               int value) {
  unsigned int program = getShader(shaderName);
  if (program == 0) return;
  glUseProgram(program);
  glUniform1i(glGetUniformLocation(program, uniformName.c_str()), value);
}

void ShaderManager::setUniform(const std::string &shaderName,
                               const std::string &uniformName,
                               float value) {
  unsigned int program = getShader(shaderName);
  if (program == 0) return;
  glUseProgram(program);
  glUniform1f(glGetUniformLocation(program, uniformName.c_str()), value);
}

void ShaderManager::setUniform(const std::string &shaderName,
                               const std::string &uniformName,
                               const glm::mat4 &matrix) {
  unsigned int program = getShader(shaderName);
  if (program == 0) return;
  glUseProgram(program);
  glUniformMatrix4fv(glGetUniformLocation(program, uniformName.c_str()), 1, GL_FALSE, &matrix[0][0]);
}

bool ShaderManager::checkLinkErrors(unsigned int program) const {
  int success;
  char infoLog[1024];
  glGetProgramiv(program, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(program, 1024, nullptr, infoLog);
    LOG_ERROR("Shader program linking error: ", infoLog);
    return false;
  }
  return true;
}
