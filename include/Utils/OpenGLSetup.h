#ifndef OPENGLSETUP_H
#define OPENGLSETUP_H

// Check if GLFW was included first (which is wrong)
#ifdef _GLFW3_H
#error "GLFW was included before GLAD! Include OpenGLSetup.h instead."
#endif

// Ensure GLAD is included first
#include <glad/gl.h>
#include <GLFW/glfw3.h>

#endif // OPENGLSETUP_H
