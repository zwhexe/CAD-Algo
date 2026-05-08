cmake_minimum_required(VERSION 3.20)
project(NurbsViewer VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找依赖
find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)

# GLAD - OpenGL 加载器
add_library(glad STATIC
    ${CMAKE_SOURCE_DIR}/external/glad/src/glad.c
)
target_include_directories(glad PUBLIC 
    ${CMAKE_SOURCE_DIR}/external/glad/include
)

# Core 数学和曲线库
add_library(nurbs_core STATIC
    core/math/point.cpp
    core/curves/bezier_curve.cpp
    core/surfaces/bezier_surface.cpp
)
target_include_directories(nurbs_core PUBLIC ${CMAKE_SOURCE_DIR})

# Viewer 库
add_library(viewer STATIC
    viewer/window.cpp
    viewer/shader.cpp
    viewer/curve_renderer.cpp
)
target_include_directories(viewer PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(viewer PUBLIC nurbs_core glad glfw OpenGL::GL ${CMAKE_DL_LIBS})

# Examples - Chapter 1 only
add_executable(ex_bezier_decasteljau examples/ex_bezier_decasteljau.cpp)
target_link_libraries(ex_bezier_decasteljau PRIVATE viewer)

# 后续章节示例（暂时注释）
# add_executable(ex_bspline_curve examples/ex_bspline_curve.cpp)
# target_link_libraries(ex_bspline_curve PRIVATE viewer)
# add_executable(ex_nurbs_curve examples/ex_nurbs_curve.cpp)
# target_link_libraries(ex_nurbs_curve PRIVATE viewer)

# 复制 shader 文件到构建目录
file(COPY ${CMAKE_SOURCE_DIR}/viewer/shaders DESTINATION ${CMAKE_BINARY_DIR})

message(STATUS "NurbsViewer configured successfully")
message(STATUS "Build examples with: cmake --build build")
