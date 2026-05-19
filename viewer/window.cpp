#include "viewer/window.hpp"
#include <stdexcept>
#include <iostream>

namespace viewer 
{

Window::Window(int width, int height, const std::string& title)
    : m_width(width), m_height(height)
{
    // 初始化 GLFW
    if (!glfwInit()) 
        throw std::runtime_error("Failed to initialize GLFW");
    else
        std::cout << "========== GLFW initialized successfully ==========\n" << std::endl;

    // OpenGL 版本设置
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4); // MSAA 抗锯齿
    
    // 创建窗口
    m_window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!m_window) 
    {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }
    
    glfwMakeContextCurrent(m_window);
    
    // 加载 OpenGL 函数
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) 
    {
        glfwTerminate();
        throw std::runtime_error("Failed to initialize GLAD");
    }
    
    // 设置用户指针，用于回调
    glfwSetWindowUserPointer(m_window, this);
    
    // 注册回调
    glfwSetFramebufferSizeCallback(m_window, framebufferSizeCallback);
    glfwSetCursorPosCallback(m_window, cursorPosCallback);
    glfwSetMouseButtonCallback(m_window, mouseButtonCallback);
    glfwSetScrollCallback(m_window, scrollCallback);
    glfwSetKeyCallback(m_window, keyCallback);
    
    // 启用 MSAA
    glEnable(GL_MULTISAMPLE);
    
    // 启用混合
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // 设置视口
    glViewport(0, 0, width, height);
    
    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;
}

Window::~Window() 
{
    if (m_window) 
        glfwDestroyWindow(m_window);
    glfwTerminate();
}

bool Window::shouldClose() const 
{
    return glfwWindowShouldClose(m_window);
}

void Window::pollEvents() 
{
    glfwPollEvents();
}

void Window::swapBuffers() 
{
    glfwSwapBuffers(m_window);
}

void Window::getMousePos(double& x, double& y) const 
{
    glfwGetCursorPos(m_window, &x, &y);
}

bool Window::isMouseButtonPressed(int button) const 
{
    return glfwGetMouseButton(m_window, button) == GLFW_PRESS;
}

void Window::framebufferSizeCallback(GLFWwindow* window, int width, int height) 
{
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(window));
    self->m_width = width;
    self->m_height = height;
    glViewport(0, 0, width, height);
    if (self->m_resizeCallback) 
    {
        self->m_resizeCallback(width, height);
    }
}

void Window::cursorPosCallback(GLFWwindow* window, double xpos, double ypos) 
{
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(window));
    if (self->m_mouseMoveCallback) 
    {
        self->m_mouseMoveCallback(xpos, ypos);
    }
}

void Window::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) 
{
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(window));
    if (self->m_mouseButtonCallback) 
    {
        self->m_mouseButtonCallback(button, action, mods);
    }
}

void Window::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) 
{
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(window));
    if (self->m_scrollCallback) 
    {
        self->m_scrollCallback(xoffset, yoffset);
    }
}

void Window::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) 
{
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(window));
    
    // ESC 关闭窗口
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    
    if (self->m_keyCallback) 
        self->m_keyCallback(key, scancode, action, mods);
}

} // namespace viewer
