/**
 * B-spline 曲线可视化示例
 * 
 * 基于 The NURBS Book 第2章
 * 
 * 展示内容:
 * - B-spline 基函数
 * - de Boor 算法的几何意义
 * - 节点向量的影响
 * - 节点插入
 * 
 * 操作说明:
 * - 鼠标左键拖拽: 移动控制点
 * - 鼠标滚轮: 调整参数 u
 * - 空格键: 切换 de Boor 中间过程显示
 * - K 键: 在当前 u 处插入节点
 * - 1-4: 切换曲线次数
 * - R 键: 重置到初始状态
 * - ESC: 退出
 */

#include "viewer/window.hpp"
#include "viewer/curve_renderer.hpp"
#include "core/curves/bspline_curve.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>

// 应用状态
struct AppState 
{
    nurbs::PointList controlPoints;
    std::vector<double> knots;
    int degree = 3;
    double u = 0.5;
    bool showDeBoor = true;
    int selectedPoint = -1;
    
    // 视图参数
    float viewLeft = -0.5f;
    float viewRight = 5.5f;
    float viewBottom = -0.5f;
    float viewTop = 3.5f;
};

// 屏幕坐标转世界坐标
nurbs::Point screenToWorld(double sx, double sy, int width, int height, const AppState& state) 
{
    double nx = sx / width;
    double ny = 1.0 - sy / height;
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

std::string formatKnots(const std::vector<double>& knots) 
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "[";
    for (size_t i = 0; i < knots.size(); i++) {
        if (i > 0) oss << ", ";
        oss << knots[i];
    }
    oss << "]";
    return oss.str();
}

void printStatus(const AppState& state) 
{
    std::cout << "\n--- B-spline Status ---" << std::endl;
    std::cout << "Degree: " << state.degree << std::endl;
    std::cout << "Control Points: " << state.controlPoints.size() << std::endl;
    std::cout << "Knots: " << formatKnots(state.knots) << std::endl;
    std::cout << "u = " << std::fixed << std::setprecision(3) << state.u << std::endl;
}

void printUsage() 
{
    std::cout << "\n============================================" << std::endl;
    std::cout << "  B-spline Curve Visualization" << std::endl;
    std::cout << "  The NURBS Book - Chapter 2" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "\nControls:" << std::endl;
    std::cout << "  Left Mouse Drag  - Move control points" << std::endl;
    std::cout << "  Scroll Wheel     - Adjust parameter u" << std::endl;
    std::cout << "  Space            - Toggle de Boor visualization" << std::endl;
    std::cout << "  K                - Insert knot at current u" << std::endl;
    std::cout << "  1, 2, 3, 4       - Set curve degree" << std::endl;
    std::cout << "  R                - Reset to initial state" << std::endl;
    std::cout << "  ESC              - Exit" << std::endl;
    std::cout << "\nColor Legend:" << std::endl;
    std::cout << "  Gray   - Control polygon" << std::endl;
    std::cout << "  Blue   - de Boor Level 1" << std::endl;
    std::cout << "  Green  - de Boor Level 2" << std::endl;
    std::cout << "  Orange - de Boor Level 3" << std::endl;
    std::cout << "  Black  - B-spline curve" << std::endl;
    std::cout << "  Red    - Point on curve at u" << std::endl;
    std::cout << std::endl;
}

void resetState(AppState& state) 
{
    // 6 个控制点
    state.controlPoints = {
        {0.0, 0.0, 0.0},
        {1.0, 2.0, 0.0},
        {2.0, 2.5, 0.0},
        {3.5, 1.5, 0.0},
        {4.5, 2.0, 0.0},
        {5.0, 0.5, 0.0}
    };
    state.degree = 3;
    state.u = 0.5;
    
    // 让 BSplineCurve 自动生成 clamped uniform 节点向量
    nurbs::BSplineCurve tempCurve(state.controlPoints, state.degree);
    state.knots = tempCurve.knots();
}

int main() 
{
    printUsage();
    
    // 创建窗口
    viewer::Window window(1024, 768, "B-spline Curve - The NURBS Book");
    
    // 创建渲染器
    viewer::CurveRenderer renderer;
    
    // 初始化应用状态
    AppState state;
    resetState(state);
    
    // 设置视图
    renderer.setView(state.viewLeft, state.viewRight, state.viewBottom, state.viewTop);
    
    printStatus(state);
    
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
        // 限制在有效参数范围内
        nurbs::BSplineCurve curve(state.controlPoints, state.degree, state.knots);
        state.u = std::max(curve.uMin(), std::min(curve.uMax(), state.u));
        std::cout << "u = " << std::fixed << std::setprecision(3) << state.u << std::endl;
    });
    
    // 键盘回调
    window.setKeyCallback([&](int key, int scancode, int action, int mods) {
        if (action == GLFW_PRESS) {
            if (key == GLFW_KEY_SPACE) {
                state.showDeBoor = !state.showDeBoor;
                std::cout << "de Boor visualization: " 
                          << (state.showDeBoor ? "ON" : "OFF") << std::endl;
            }
            else if (key == GLFW_KEY_K) {
                // 节点插入
                try {
                    nurbs::BSplineCurve curve(state.controlPoints, state.degree, state.knots);
                    auto newCurve = curve.knotInsert(state.u);
                    state.controlPoints = newCurve.controlPoints();
                    state.knots = newCurve.knots();
                    std::cout << "Inserted knot at u = " << state.u << std::endl;
                    printStatus(state);
                } catch (const std::exception& e) {
                    std::cerr << "Knot insertion failed: " << e.what() << std::endl;
                }
            }
            else if (key >= GLFW_KEY_1 && key <= GLFW_KEY_4) {
                int newDegree = key - GLFW_KEY_1 + 1;
                int n = static_cast<int>(state.controlPoints.size()) - 1;
                if (newDegree <= n) {
                    state.degree = newDegree;
                    // 重新生成节点向量
                    nurbs::BSplineCurve tempCurve(state.controlPoints, state.degree);
                    state.knots = tempCurve.knots();
                    std::cout << "Degree changed to " << state.degree << std::endl;
                    printStatus(state);
                } else {
                    std::cout << "Degree " << newDegree 
                              << " too high for " << (n+1) << " control points" << std::endl;
                }
            }
            else if (key == GLFW_KEY_R) {
                resetState(state);
                std::cout << "Reset to initial state" << std::endl;
                printStatus(state);
            }
        }
    });
    
    // 窗口大小回调
    window.setResizeCallback([&](int width, int height) {
        float aspect = static_cast<float>(width) / height;
        float centerX = (state.viewLeft + state.viewRight) / 2;
        float centerY = (state.viewBottom + state.viewTop) / 2;
        float halfHeight = (state.viewTop - state.viewBottom) / 2;
        float halfWidth = halfHeight * aspect;
        
        state.viewLeft = centerX - halfWidth;
        state.viewRight = centerX + halfWidth;
        renderer.setView(state.viewLeft, state.viewRight, state.viewBottom, state.viewTop);
    });
    
    // 主循环
    while (!window.shouldClose()) 
    {
        // 清屏
        glClearColor(0.95f, 0.95f, 0.95f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        
        try {
            // 创建 B-spline 曲线
            nurbs::BSplineCurve curve(state.controlPoints, state.degree, state.knots);
            
            // 绘制控制多边形
            renderer.drawControlPolygon(state.controlPoints, 0.5f, 0.5f, 0.5f);
            
            // 绘制 de Boor 中间过程
            if (state.showDeBoor) 
            {
                auto pyramid = curve.deBoorPyramid(state.u);
                renderer.drawDeCasteljauPyramid(pyramid);  // 复用 Bezier 的绘制函数
            }
            
            // 绘制曲线
            auto curvePoints = curve.sample(200);
            renderer.drawCurve(curvePoints, 0.1f, 0.1f, 0.1f, 3.0f);
            
            // 绘制控制点
            renderer.drawPoints(state.controlPoints, 0.2f, 0.4f, 0.8f, 12.0f);
            
            // 高亮当前点
            nurbs::PointList highlight = {curve.evaluate(state.u)};
            renderer.drawPoints(highlight, 1.0f, 0.2f, 0.2f, 16.0f);
            
        } catch (const std::exception& e) {
            // 发生错误时显示空白
        }
        
        window.swapBuffers();
        window.pollEvents();
    }
    
    return 0;
}
