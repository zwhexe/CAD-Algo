#pragma once

#include <glad/glad.h>
#include "core/math/point.hpp"
#include "core/curves/bezier_curve.hpp"
#include "viewer/shader.hpp"
#include <memory>
#include <vector>

namespace viewer 
{

/**
 * 曲线渲染器
 * 负责将 Bezier 曲线渲染到屏幕上
 */
class CurveRenderer 

{
public:
    CurveRenderer();
    ~CurveRenderer();
    
    // 禁止拷贝
    CurveRenderer(const CurveRenderer&) = delete;
    CurveRenderer& operator=(const CurveRenderer&) = delete;
    
    /**
     * 设置视图参数（正交投影）
     */
    void setView(float left, float right, float bottom, float top);
    
    /**
     * 渲染完整的 Bezier 曲线场景
     * 包括：曲线、控制多边形、控制点、de Casteljau 中间过程
     */
    void render(const nurbs::BezierCurve& curve, double u, bool showDeCasteljau = true);
    
    /**
     * 渲染曲线（采样点连线）
     */
    void drawCurve(const nurbs::PointList& points, float r, float g, float b, float lineWidth = 2.0f);
    
    /**
     * 渲染控制多边形
     */
    void drawControlPolygon(const nurbs::PointList& points, float r, float g, float b);
    
    /**
     * 渲染控制点
     */
    void drawPoints(const nurbs::PointList& points, float r, float g, float b, float size = 10.0f);
    
    /**
     * 渲染 de Casteljau 金字塔（中间插值过程）
     */
    void drawDeCasteljauPyramid(const std::vector<nurbs::PointList>& pyramid);
    
private:
    std::unique_ptr<Shader> m_shader;
    
    // VAO/VBO 用于动态绘制
    GLuint m_lineVAO, m_lineVBO;
    GLuint m_pointVAO, m_pointVBO;
    
    // 投影矩阵 (正交投影，存为列主序)
    float m_projection[16];
    
    void initBuffers();
    void updateProjection(float left, float right, float bottom, float top);
};

} // namespace viewer
