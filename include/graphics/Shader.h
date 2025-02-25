#ifndef SHADER_H
#define SHADER_H
#include <string>

#include "utils/OpenGLSetup.h"

class Shader {
public:
  Shader(const std::string& vertexPath, const std::string& fragmentPath);
  ~Shader();

  void use() const;

  GLuint getID() const;

private:
  GLuint m_ID;

  GLuint createShaderProgram(const std::string& vertexSource, const std::string& fragmentSource);

  GLuint compileShader(const std::string &source, GLenum type);

  std::string loadShaderFile(const std::string &path);
};

#endif // SHADER_H
