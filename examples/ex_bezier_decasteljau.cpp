/**
 * de Casteljau 算法可视化示例
 * 
 * 基于 The NURBS Book 第1.3节
 * 
 * 操作说明:
 * - 鼠标左键拖拽: 移动控制点
 * - 鼠标滚轮: 调整参数 u
 * - 空格键: 切换 de Casteljau 中间过程显示
 * - R 键: 重置到初始状态
 * - ESC: 退出
 */

#include "viewer/window.hpp"
#include "viewer/curve_renderer.hpp"
#include "core/curves/bezier_curve.hpp"
#include <iostream>
#include <cmath>

// 应用状态
struct AppState 
{
    nurbs::PointList controlPoints;
    double u = 0.5;
    bool showDeCasteljau = true;
    int selectedPoint = -1;
    
    // 视图参数
    float viewLeft = -0.5f;
    float viewRight = 4.5f;
    float viewBottom = -0.5f;
    float viewTop = 3.0f;
};

// 屏幕坐标转世界坐标
nurbs::Point screenToWorld(double sx, double sy, int width, int height, const AppState& state) 
{
    double nx = sx / width;
    double ny = 1.0 - sy / height;  // 翻转 Y
    double wx = state.viewLeft + nx * (state.viewRight - state.viewLeft);
    double wy = state.viewBottom + ny * (state.viewTop - state.viewBottom); 
    return {wx, wy, 0};
}

// 查找最近的控制点
int findNearestPoint(const nurbs::PointList& points, const nurbs::Point& target, double threshold) 
{
    int nearest = -1;
    double minDist = threshold;
    for (size_t i = 0; i < points.size(); i++) 
    {
        double dist = (points[i] - target).length();
        if (dist < minDist) 
        {
            minDist = dist;
            nearest = static_cast<int>(i);
        }
    } 
    return nearest;
}

void printUsage() 
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "  de Casteljau Algorithm Visualization" << std::endl;
    std::cout << "  The NURBS Book - Section 1.3" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "\nControls:" << std::endl;
    std::cout << "  Left Mouse Drag  - Move control points" << std::endl;
    std::cout << "  Scroll Wheel     - Adjust parameter u" << std::endl;
    std::cout << "  Space            - Toggle de Casteljau visualization" << std::endl;
    std::cout << "  R                - Reset to initial state" << std::endl;
    std::cout << "  ESC              - Exit" << std::endl;
    std::cout << "\nColor Legend:" << std::endl;
    std::cout << "  Gray   - Control polygon" << std::endl;
    std::cout << "  Blue   - de Casteljau Level 1" << std::endl;
    std::cout << "  Green  - de Casteljau Level 2" << std::endl;
    std::cout << "  Orange - de Casteljau Level 3" << std::endl;
    std::cout << "  Black  - Bezier curve" << std::endl;
    std::cout << "  Red    - Point on curve at u" << std::endl;
    std::cout << std::endl;
}

int main() 
{
    printUsage();
    
    // 创建窗口
    viewer::Window window(1024, 768, "de Casteljau Algorithm - The NURBS Book");
    
    // 创建渲染器
    viewer::CurveRenderer renderer;
    
    // 初始化应用状态
    AppState state;
    state.controlPoints = {
        {0.0, 0.0, 0.0},  // P0
        {1.0, 2.0, 0.0},  // P1
        {3.0, 2.0, 0.0},  // P2
        {4.0, 0.0, 0.0}   // P3
    };
    
    // 设置视图
    renderer.setView(state.viewLeft, state.viewRight, state.viewBottom, state.viewTop);
    
    // 鼠标按钮回调
    window.setMouseButtonCallback([&](int button, int action, int mods) {
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            if (action == GLFW_PRESS) {
                double mx, my;
                window.getMousePos(mx, my);
                nurbs::Point worldPos = screenToWorld(mx, my, window.width(), window.height(), state);
                state.selectedPoint = findNearestPoint(state.controlPoints, worldPos, 0.3);
            } else {
                state.selectedPoint = -1;
            }
        }
    });
    
    // 鼠标移动回调
    window.setMouseMoveCallback([&](double x, double y) {
        if (state.selectedPoint >= 0) {
            nurbs::Point worldPos = screenToWorld(x, y, window.width(), window.height(), state);
            state.controlPoints[state.selectedPoint] = worldPos;
        }
    });
    
    // 滚轮回调 - 调整 u 值
    window.setScrollCallback([&](double xoffset, double yoffset) {
        state.u += yoffset * 0.05;
        state.u = std::max(0.0, std::min(1.0, state.u));
        std::cout << "u = " << state.u << std::endl;
    });
    
    // 键盘回调
    window.setKeyCallback([&](int key, int scancode, int action, int mods) {
        if (action == GLFW_PRESS) {
            if (key == GLFW_KEY_SPACE) {
                state.showDeCasteljau = !state.showDeCasteljau;
                std::cout << "de Casteljau visualization: " 
                          << (state.showDeCasteljau ? "ON" : "OFF") << std::endl;
            }
            else if (key == GLFW_KEY_R) {
                // 重置
                state.controlPoints = 
                {
                    {0.0, 0.0, 0.0},
                    {1.0, 2.0, 0.0},
                    {3.0, 2.0, 0.0},
                    {4.0, 0.0, 0.0}
                };
                state.u = 0.5;
                std::cout << "Reset to initial state" << std::endl;
            }
        }
    });
    
    // 窗口大小回调
    window.setResizeCallback([&](int width, int height) {
        // 保持宽高比
        float aspect = static_cast<float>(width) / height;
        float centerX = (state.viewLeft + state.viewRight) / 2;
        float centerY = (state.viewBottom + state.viewTop) / 2;
        float halfHeight = (state.viewTop - state.viewBottom) / 2;
        float halfWidth = halfHeight * aspect;
        
        state.viewLeft = centerX - halfWidth;
        state.viewRight = centerX + halfWidth;
        renderer.setView(state.viewLeft, state.viewRight, state.viewBottom, state.viewTop);
    });
    
    std::cout << "Initial u = " << state.u << std::endl;
    
    // 主循环
    while (!window.shouldClose()) 
    {
        // 清屏 - 浅灰色背景
        glClearColor(0.95f, 0.95f, 0.95f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        
        // 创建曲线并渲染
        nurbs::BezierCurve curve(state.controlPoints);
        renderer.render(curve, state.u, state.showDeCasteljau);
        
        window.swapBuffers();
        window.pollEvents();
    }
    
    return 0;
}
