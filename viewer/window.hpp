#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <string>
#include <functional>

namespace viewer 
{

/**
 * OpenGL 窗口封装
 */
class Window 
{
public:
    Window(int width, int height, const std::string& title);
    ~Window();
    
    // 禁止拷贝
    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;
    
    // 主循环
    bool shouldClose() const;
    void pollEvents();
    void swapBuffers();
    
    // 窗口信息
    int width() const { return m_width; }
    int height() const { return m_height; }
    float aspectRatio() const { return static_cast<float>(m_width) / m_height; }
    GLFWwindow* handle() const { return m_window; }
    
    // 回调设置
    using ResizeCallback = std::function<void(int, int)>;
    using MouseCallback = std::function<void(double, double)>;
    using MouseButtonCallback = std::function<void(int, int, int)>;
    using ScrollCallback = std::function<void(double, double)>;
    using KeyCallback = std::function<void(int, int, int, int)>;
    
    void setResizeCallback(ResizeCallback cb) { m_resizeCallback = cb; }
    void setMouseMoveCallback(MouseCallback cb) { m_mouseMoveCallback = cb; }
    void setMouseButtonCallback(MouseButtonCallback cb) { m_mouseButtonCallback = cb; }
    void setScrollCallback(ScrollCallback cb) { m_scrollCallback = cb; }
    void setKeyCallback(KeyCallback cb) { m_keyCallback = cb; }
    
    // 鼠标状态
    void getMousePos(double& x, double& y) const;
    bool isMouseButtonPressed(int button) const;
    
private:
    GLFWwindow* m_window = nullptr;
    int m_width, m_height;
    
    // 回调函数
    ResizeCallback m_resizeCallback;
    MouseCallback m_mouseMoveCallback;
    MouseButtonCallback m_mouseButtonCallback;
    ScrollCallback m_scrollCallback;
    KeyCallback m_keyCallback;
    
    // GLFW 回调的静态包装
    static void framebufferSizeCallback(GLFWwindow* window, int width, int height);
    static void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
};

} // namespace viewer
