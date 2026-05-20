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
    template<typename TCurve>
    void render(const TCurve& curve, double u, bool showDeCasteljau = true)
    {
        m_shader->use();
        m_shader->setMat4("projection", m_projection); 

        drawControlPolygon(curve.controlPoints(), 0.5f, 0.5f, 0.5f);
        if (showDeCasteljau && u >= 0.0 && u <= 1.0)
        {
            auto pyramid = curve.deCasteljauPyramid(u);
            drawDeCasteljauPyramid(pyramid);
        }

        auto curvePoints = curve.sample(100);
        drawCurve(curvePoints, 0.1f, 0.1f, 0.1f, 3.0f);
        drawPoints(curve.controlPoints(), 0.2f, 0.4f, 0.8f, 12.0f);
        
        if(u >= 0.0 && u <= 1.0) 
        {
            nurbs::PointList highlight = {curve.evaluate(u)};
            drawPoints(highlight, 1.0f, 0.2f, 0.2f, 16.0f);
        }
    }
    
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
