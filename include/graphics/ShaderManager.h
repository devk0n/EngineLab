#ifndef SHADERMANAGER_H
#define SHADERMANAGER_H

#include <mutex>
#include <string>
#include <unordered_map>

#include <glm/glm.hpp>
#include "utils/OpenGLSetup.h"

class ShaderManager {
public:
  ShaderManager() = default;
  ~ShaderManager();

  bool loadShader(const std::string &name,
                  const std::string &vertexPath,
                  const std::string &fragmentPath,
                  const std::string &geometryPath = "");

  bool loadComputeShader(const std::string &name,
                         const std::string &computePath);

  unsigned int getShader(const std::string &name) const;

  // Set uniforms
  void setUniform(const std::string &shaderName,
                  const std::string &uniformName,
                  int value) const;

  void setUniform(const std::string &shaderName,
                  const std::string &uniformName,
                  float value) const;

  void setUniform(const std::string &shaderName,
                  const std::string &uniformName,
                  const glm::mat4 &matrix) const;

  void cleanup();

private:
  std::unordered_map<std::string, unsigned int> m_shaders;
  mutable std::mutex m_mutex;

  static std::string readFile(const std::string &filePath);

  static unsigned int compileShader(GLenum type, const std::string &source);

  static bool linkProgram(unsigned int program,
                          unsigned int vertexShader,
                          unsigned int fragmentShader,
                          unsigned int geometryShader = 0);

  static bool checkCompileErrors(unsigned int shader, const std::string &type);
  static bool checkLinkErrors(unsigned int program);
};

#endif // SHADERMANAGER_H
