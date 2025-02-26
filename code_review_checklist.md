# Code Review Checklist

## 1. Code Structure & Organization
- [ ] Break `Application` initialization into separate methods
- [ ] Consider using a precompiled header (`pch.h`) to reduce compilation time

## 2. Performance Optimizations
- [ ] Optimize `SceneManager` to allow scene caching instead of always reloading
- [ ] Use `std::vector` instead of `std::stack` for scene storage
- [ ] Reduce redundant `glBindVertexArray(0)` calls
- [ ] Optimize shader file reading (`readFile()`) by considering memory-mapped files

## 3. Rendering Pipeline & OpenGL Usage
- [ ] Avoid disabling/re-enabling depth testing every frame in `renderSky()`
- [ ] Allow user-defined background colors instead of hardcoded `glClearColor()`
- [ ] Optimize shader loading to avoid synchronous loads at startup

## 4. Input Handling
- [ ] Implement event-driven scene switching instead of checking `GLFW_KEY_ENTER` every frame
- [ ] Apply mouse smoothing for better camera movement in `processMouseMovement()`
- [ ] Clamp movement speed modifications in `processScroll()`

## 5. Logging System
- [ ] Improve shader compilation error messages by printing actual source lines
- [ ] Ensure `Logger::initialize()` allows console-only use without requiring file output

## 6. Camera System
- [ ] Avoid gimbal lock in `lookAt()` by improving yaw/pitch calculations
- [ ] Consider returning `const glm::vec3&` instead of copying in `getPosition()`

## 7. Scene Management
- [ ] Implement scene caching for frequently accessed scenes
- [ ] Prevent duplicate scenes from being added in `pushScene()`
- [ ] Ensure `popScene()` does not unload resources still in use

## 8. Shaders & Graphics
- [ ] Allow runtime configuration for grid size instead of hardcoding 1000m × 1000m
- [ ] Allow runtime configuration for star density instead of `STAR_DENSITY 0.02`
- [ ] Optimize `glUniform1f(glGetUniformLocation(shader, "u_time"), glfwGetTime());` to update only when necessary

## 9. Miscellaneous
- [ ] Reduce unnecessary `glBindVertexArray(0)` calls
- [ ] Review OpenGL state changes to avoid redundant operations
- [ ] Review and test fog blending in grid shader

---
Mark items as completed `[x]` as you go.
