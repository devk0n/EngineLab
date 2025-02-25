#ifndef SHADERMANAGER_H
#define SHADERMANAGER_H

#include <string>
#include <unordered_map>

#include "utils/OpenGLSetup.h"

class ShaderManager {
public:
  ShaderManager() = default;
  ~ShaderManager();

  bool loadShader(const std::string& name,
                  const std::string& vertexPath,
                  const std::string& fragmentPath);
  unsigned int getShader(const std::string& name) const;
  void cleanup();

private:
  std::unordered_map<std::string, unsigned int> m_shaders;

  std::string readFile(const std::string& filePath) const;
  unsigned int compileShader(GLenum type, const std::string& source) const;
  bool linkProgram(unsigned int program,
                   unsigned int vertexShader,
                   unsigned int fragmentShader) const;
};


#endif // SHADERMANAGER_H
