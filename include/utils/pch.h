#ifndef PCH_H
#define PCH_H

// Standard headers
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <stack>
#include <unordered_set>
#include <fstream>
#include <ranges>
#include <sstream>

// 3rd-party (like Eigen)
#include <Eigen/Dense>

// Check if GLFW was included first (which is wrong)
#ifdef _GLFW3_H
#error "GLFW was included before GLAD! Include OpenGLSetup.h instead."
#endif

// Ensure GLAD is included first
#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.inl>

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "Logger.h"

#endif // PCH_H
