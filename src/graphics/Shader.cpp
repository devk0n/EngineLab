#include "graphics/Shader.h"
#include "utils/Logger.h"

Shader::Shader(const std::string &vertexPath, const std::string &fragmentPath) {
  std::string vertexSource = loadShaderFile(vertexPath);
  std::string fragmentSource = loadShaderFile(fragmentPath);
  m_ID = createShaderProgram(vertexSource, fragmentSource);
}

Shader::~Shader() {
  glDeleteProgram(m_ID);
}

void Shader::use() const {
  glUseProgram(m_ID);
}

GLuint Shader::getID() const {
  return m_ID;
}

GLuint Shader::createShaderProgram(const std::string &vertexSource,
                                   const std::string &fragmentSource) {

  GLuint vertexShader = compileShader(vertexSource, GL_VERTEX_SHADER);
  GLuint fragmentShader = compileShader(fragmentSource, GL_FRAGMENT_SHADER);

  GLuint program = glCreateProgram();
  glAttachShader(program, vertexShader);
  glAttachShader(program, fragmentShader);
  glLinkProgram(program);

  GLint success;
  glGetProgramiv(program, GL_LINK_STATUS, &success);
  if (!success) {
    char infoLog[512];
    glGetProgramInfoLog(program, 512, nullptr, infoLog);
    LOG_ERROR("Shader program linking failed" + std::string(infoLog));
  }

  glDeleteShader(vertexShader);
  glDeleteShader(fragmentShader);

  return program;
}

GLuint Shader::compileShader(const std::string& source, GLenum type) {
  GLuint shader = glCreateShader(type);
  const char* src = source.c_str();
  glShaderSource(shader, 1, &src, nullptr);
  glCompileShader(shader);

  GLint success;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
  if (!success) {
    char infoLog[512];
    glGetShaderInfoLog(shader, 512, nullptr, infoLog);
    LOG_ERROR("Shader compilation failed" + std::string(infoLog));
  }

  return shader;
}

std::string Shader::loadShaderFile(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    LOG_ERROR("Failed to open shader file: " + path);
    return "";
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}
