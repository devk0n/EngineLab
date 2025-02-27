cmake_minimum_required(VERSION 3.30)
project(EngineLab)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static")

# Include FetchContent module
include(FetchContent)

# Fetch Eigen
FetchContent_Declare(
        Eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG master
        GIT_SHALLOW TRUE
)
set(EIGEN_BUILD_DOC OFF)
set(EIGEN_BUILD_PKGCONFIG OFF)
FetchContent_MakeAvailable(Eigen)

# Fetch GLFW
FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG 3.4
)
set(GLFW_BUILD_DOCS OFF)
set(GLFW_BUILD_EXAMPLES OFF)
set(GLFW_BUILD_TESTS OFF)
FetchContent_MakeAvailable(glfw)

# Add GLAD as a static library
add_library(glad STATIC external/glad/src/glad.c)
target_include_directories(glad PUBLIC external/glad/include)

# Fetch ImGui
FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.91.7
)
FetchContent_MakeAvailable(imgui)
add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui glfw OpenGL::GL glad)

# Fetch GLM
FetchContent_Declare(
        glm
        GIT_REPOSITORY https://github.com/g-truc/glm.git
        GIT_TAG 1.0.1
)
FetchContent_MakeAvailable(glm)

# Find OpenGL
find_package(OpenGL REQUIRED)

# Source files
set(SOURCES
        src/main.cpp

        src/core/Application.cpp
        src/core/WindowManager.cpp
        src/core/InputManager.cpp
        src/core/Renderer.cpp
        src/core/ImGuiManager.cpp
        src/core/EnvironmentManager.cpp
        src/core/Camera.cpp
        src/core/ShaderManager.cpp

        src/environments/Dashboard.cpp
        src/environments/Simulation.cpp
)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Apply Warning Flags Only to Your Own Project Code
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Werror -Wpedantic)

# Link libraries to the project
target_link_libraries(${PROJECT_NAME} PRIVATE
        imgui
        glfw
        glad
        glm::glm
        OpenGL::GL
        Eigen3::Eigen
)

# Include the current source directory in the project's include directories
target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src

)