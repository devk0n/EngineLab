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
                  int value);

  void setUniform(const std::string &shaderName,
                  const std::string &uniformName,
                  float value);

  void setUniform(const std::string &shaderName,
                  const std::string &uniformName,
                  const glm::mat4 &matrix);

  void cleanup();

private:
  std::unordered_map<std::string, unsigned int> m_shaders;
  mutable std::mutex m_mutex;

  std::string readFile(const std::string &filePath) const;

  unsigned int compileShader(GLenum type, const std::string &source) const;

  bool linkProgram(unsigned int program,
                   unsigned int vertexShader,
                   unsigned int fragmentShader,
                   unsigned int geometryShader = 0) const;

  bool checkCompileErrors(unsigned int shader, const std::string &type) const;
  bool checkLinkErrors(unsigned int program) const;
};

#endif // SHADERMANAGER_H
