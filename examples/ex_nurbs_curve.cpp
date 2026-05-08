/**
 * NURBS 曲线可视化示例
 * 
 * 基于 The NURBS Book 第4章
 * 
 * 展示内容:
 * - NURBS 曲线的有理性质
 * - 权重对曲线形状的影响
 * - 精确圆和圆弧的表示
 * 
 * 操作说明:
 * - 鼠标左键拖拽: 移动控制点
 * - 鼠标滚轮: 调整选中点的权重
 * - 右键点击: 选择控制点以调整权重
 * - 空格键: 切换显示模式
 * - 1: 显示自定义 NURBS 曲线
 * - 2: 显示圆弧
 * - 3: 显示椭圆
 * - R 键: 重置
 * - ESC: 退出
 */

#include "viewer/window.hpp"
#include "viewer/curve_renderer.hpp"
#include "core/curves/nurbs_curve.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

enum class DisplayMode 
{
    Custom,
    Arc,
    Ellipse
};

struct AppState 
{
    nurbs::WeightedPointList controlPoints;
    std::vector<double> knots;
    int degree = 2;
    double u = 0.5;
    int selectedPoint = -1;
    int weightEditPoint = -1;  // 正在编辑权重的点
    DisplayMode mode = DisplayMode::Custom;
    
    float viewLeft = -2.0f;
    float viewRight = 4.0f;
    float viewBottom = -1.5f;
    float viewTop = 3.5f;
};

nurbs::Point screenToWorld(double sx, double sy, int width, int height, const AppState& state) 
{
    double nx = sx / width;
    double ny = 1.0 - sy / height;
    double wx = state.viewLeft + nx * (state.viewRight - state.viewLeft);
    double wy = state.viewBottom + ny * (state.viewTop - state.viewBottom);
    return {wx, wy, 0};
}

int findNearestPoint(const nurbs::WeightedPointList& points, const nurbs::Point& target, double threshold) 
{
    int nearest = -1;
    double minDist = threshold;
    for (size_t i = 0; i < points.size(); i++) 
    {
        double dist = (points[i].point - target).length();
        if (dist < minDist) 
        {
            minDist = dist;
            nearest = static_cast<int>(i);
        }
    }
    return nearest;
}

void printWeights(const nurbs::WeightedPointList& points) 
{
    std::cout << "Weights: [";
    for (size_t i = 0; i < points.size(); i++) 
    {
        if (i > 0) std::cout << ", ";
        std::cout << std::fixed << std::setprecision(2) << points[i].weight;
    }
    std::cout << "]" << std::endl;
}

void printUsage() 
{
    std::cout << "\n============================================" << std::endl;
    std::cout << "  NURBS Curve Visualization" << std::endl;
    std::cout << "  The NURBS Book - Chapter 4" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "\nControls:" << std::endl;
    std::cout << "  Left Mouse Drag   - Move control points" << std::endl;
    std::cout << "  Right Click       - Select point for weight editing" << std::endl;
    std::cout << "  Scroll Wheel      - Adjust weight of selected point" << std::endl;
    std::cout << "  1                 - Custom NURBS curve" << std::endl;
    std::cout << "  2                 - Circle/Arc demo" << std::endl;
    std::cout << "  3                 - Ellipse demo" << std::endl;
    std::cout << "  R                 - Reset" << std::endl;
    std::cout << "  ESC               - Exit" << std::endl;
    std::cout << "\nNURBS Key Concept:" << std::endl;
    std::cout << "  Weight > 1: Point pulls curve toward itself" << std::endl;
    std::cout << "  Weight < 1: Point pushes curve away" << std::endl;
    std::cout << "  Weight = 0: Point has no influence (use with caution)" << std::endl;
    std::cout << std::endl;
}

void initCustomCurve(AppState& state) 
{
    // 二次 NURBS 曲线，5 个控制点
    state.controlPoints = {
        {{0.0, 0.0, 0.0}, 1.0},
        {{0.5, 2.0, 0.0}, 1.0},
        {{1.5, 2.5, 0.0}, 1.0},  // 这个点的权重可以调整
        {{2.5, 1.5, 0.0}, 1.0},
        {{3.0, 0.0, 0.0}, 1.0}
    };
    state.degree = 2;
    state.mode = DisplayMode::Custom;
    
    // 生成节点向量
    nurbs::NurbsCurve curve(state.controlPoints, state.degree);
    state.knots = curve.knots();
}

void initArcCurve(AppState& state) 
{
    // 创建 3/4 圆弧
    auto arc = nurbs::NurbsFactory::createArc(
        nurbs::Point(1.5, 1.5, 0), 1.5, 0, M_PI * 1.5);
    
    state.controlPoints = arc.controlPoints();
    state.knots = arc.knots();
    state.degree = arc.degree();
    state.mode = DisplayMode::Arc;
}

void initEllipseCurve(AppState& state) {
    auto ellipse = nurbs::NurbsFactory::createEllipse(
        nurbs::Point(1.5, 1.5, 0), 2.0, 1.0);
    
    state.controlPoints = ellipse.controlPoints();
    state.knots = ellipse.knots();
    state.degree = ellipse.degree();
    state.mode = DisplayMode::Ellipse;
}

int main() 
{
    printUsage();
    
    viewer::Window window(1024, 768, "NURBS Curve - The NURBS Book");
    viewer::CurveRenderer renderer;
    
    AppState state;
    initCustomCurve(state);
    
    renderer.setView(state.viewLeft, state.viewRight, state.viewBottom, state.viewTop);
    
    printWeights(state.controlPoints);
    
    // 鼠标按钮回调
    window.setMouseButtonCallback([&](int button, int action, int mods) {
        double mx, my;
        window.getMousePos(mx, my);
        nurbs::Point worldPos = screenToWorld(mx, my, window.width(), window.height(), state);
        
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            if (action == GLFW_PRESS) {
                state.selectedPoint = findNearestPoint(state.controlPoints, worldPos, 0.3);
            } else {
                state.selectedPoint = -1;
            }
        }
        else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
            int pt = findNearestPoint(state.controlPoints, worldPos, 0.3);
            if (pt >= 0) {
                state.weightEditPoint = pt;
                std::cout << "Selected point " << pt 
                          << " for weight editing (current weight: "
                          << state.controlPoints[pt].weight << ")" << std::endl;
            }
        }
    });
    
    window.setMouseMoveCallback([&](double x, double y) {
        if (state.selectedPoint >= 0) {
            nurbs::Point worldPos = screenToWorld(x, y, window.width(), window.height(), state);
            state.controlPoints[state.selectedPoint].point = worldPos;
        }
    });
    
    // 滚轮调整权重
    window.setScrollCallback([&](double xoffset, double yoffset) {
        if (state.weightEditPoint >= 0) {
            double& w = state.controlPoints[state.weightEditPoint].weight;
            w += yoffset * 0.1;
            w = std::max(0.01, std::min(10.0, w));  // 限制范围
            std::cout << "Point " << state.weightEditPoint 
                      << " weight: " << std::fixed << std::setprecision(2) << w << std::endl;
        } else {
            // 如果没有选中点，调整 u 值
            state.u += yoffset * 0.05;
            state.u = std::max(0.0, std::min(1.0, state.u));
        }
    });
    
    window.setKeyCallback([&](int key, int scancode, int action, int mods) {
        if (action == GLFW_PRESS) {
            if (key == GLFW_KEY_1) {
                initCustomCurve(state);
                std::cout << "Mode: Custom NURBS curve" << std::endl;
                printWeights(state.controlPoints);
            }
            else if (key == GLFW_KEY_2) {
                initArcCurve(state);
                std::cout << "Mode: Arc (3/4 circle)" << std::endl;
                printWeights(state.controlPoints);
            }
            else if (key == GLFW_KEY_3) {
                initEllipseCurve(state);
                std::cout << "Mode: Ellipse" << std::endl;
                printWeights(state.controlPoints);
            }
            else if (key == GLFW_KEY_R) {
                switch (state.mode) 
                {
                    case DisplayMode::Custom: initCustomCurve(state); break;
                    case DisplayMode::Arc: initArcCurve(state); break;
                    case DisplayMode::Ellipse: initEllipseCurve(state); break;
                }
                state.weightEditPoint = -1;
                std::cout << "Reset" << std::endl;
                printWeights(state.controlPoints);
            }
        }
    });
    
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
    
    while (!window.shouldClose()) 
    {
        glClearColor(0.95f, 0.95f, 0.95f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        
        try {
            nurbs::NurbsCurve curve(state.controlPoints, state.degree, state.knots);
            
            // 提取不带权重的控制点用于绘制
            nurbs::PointList pts = curve.controlPointsWithoutWeights();
            
            // 绘制控制多边形
            renderer.drawControlPolygon(pts, 0.6f, 0.6f, 0.6f);
            
            // 绘制曲线
            auto curvePoints = curve.sample(200);
            renderer.drawCurve(curvePoints, 0.1f, 0.1f, 0.1f, 3.0f);
            
            // 绘制控制点（根据权重调整大小）
            for (size_t i = 0; i < pts.size(); i++) {
                float size = 8.0f + static_cast<float>(state.controlPoints[i].weight) * 4.0f;
                
                // 正在编辑权重的点用黄色
                if (static_cast<int>(i) == state.weightEditPoint) {
                    renderer.drawPoints({pts[i]}, 1.0f, 0.8f, 0.0f, size + 4.0f);
                } else {
                    renderer.drawPoints({pts[i]}, 0.2f, 0.4f, 0.8f, size);
                }
            }
            
            // 高亮当前曲线点
            nurbs::PointList highlight = {curve.evaluate(state.u)};
            renderer.drawPoints(highlight, 1.0f, 0.2f, 0.2f, 14.0f);
            
        } catch (const std::exception& e) {
            // 曲线无效时静默处理
        }
        
        window.swapBuffers();
        window.pollEvents();
    }
    
    return 0;
}
