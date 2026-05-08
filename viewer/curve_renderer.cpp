#include "viewer/curve_renderer.hpp"
#include <cstring>

namespace viewer 
{

// 预定义颜色方案（用于 de Casteljau 各层）
static const float LAYER_COLORS[][3] = 
{
    {0.6f, 0.6f, 0.6f},  // Level 0: 灰色 (控制多边形)
    {0.2f, 0.6f, 1.0f},  // Level 1: 蓝色
    {0.2f, 0.8f, 0.4f},  // Level 2: 绿色
    {1.0f, 0.6f, 0.2f},  // Level 3: 橙色
    {0.8f, 0.2f, 0.8f},  // Level 4: 紫色
    {1.0f, 0.2f, 0.4f},  // Level 5: 红色
};
static const int NUM_COLORS = sizeof(LAYER_COLORS) / sizeof(LAYER_COLORS[0]);

CurveRenderer::CurveRenderer() 
{
    // 创建着色器
    m_shader = std::make_unique<Shader>("shaders/basic.vert", "shaders/basic.frag");
    
    initBuffers();
    
    // 默认视图
    setView(-1.0f, 5.0f, -1.0f, 4.0f);
}

CurveRenderer::~CurveRenderer() 
{
    glDeleteVertexArrays(1, &m_lineVAO);
    glDeleteBuffers(1, &m_lineVBO);
    glDeleteVertexArrays(1, &m_pointVAO);
    glDeleteBuffers(1, &m_pointVBO);
}

void CurveRenderer::initBuffers() 
{
    // 线条 VAO/VBO
    glGenVertexArrays(1, &m_lineVAO);
    glGenBuffers(1, &m_lineVBO);
    
    glBindVertexArray(m_lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_lineVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * 1000, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    // 点 VAO/VBO
    glGenVertexArrays(1, &m_pointVAO);
    glGenBuffers(1, &m_pointVBO);
    
    glBindVertexArray(m_pointVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_pointVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * 100, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    glBindVertexArray(0);
}

void CurveRenderer::setView(float left, float right, float bottom, float top) 
{
    updateProjection(left, right, bottom, top);
}

void CurveRenderer::updateProjection(float left, float right, float bottom, float top) 
{
    // 正交投影矩阵 (列主序)
    float near = -1.0f, far = 1.0f;
    std::memset(m_projection, 0, sizeof(m_projection));
    m_projection[0] = 2.0f / (right - left);
    m_projection[5] = 2.0f / (top - bottom);
    m_projection[10] = -2.0f / (far - near);
    m_projection[12] = -(right + left) / (right - left);
    m_projection[13] = -(top + bottom) / (top - bottom);
    m_projection[14] = -(far + near) / (far - near);
    m_projection[15] = 1.0f;
}

void CurveRenderer::render(const nurbs::BezierCurve& curve, double u, bool showDeCasteljau) 
{
    m_shader->use();
    m_shader->setMat4("projection", m_projection);
    
    // 1. 绘制控制多边形（虚线效果用浅色）
    drawControlPolygon(curve.controlPoints(), 0.5f, 0.5f, 0.5f);
    
    // 2. 绘制 de Casteljau 中间过程
    if (showDeCasteljau && u >= 0.0 && u <= 1.0) 
    {
        auto pyramid = curve.deCasteljauPyramid(u);
        drawDeCasteljauPyramid(pyramid);
    }
    
    // 3. 绘制曲线本身（最后绘制，覆盖在最上层）
    auto curvePoints = curve.sample(100);
    drawCurve(curvePoints, 0.1f, 0.1f, 0.1f, 3.0f);
    
    // 4. 绘制控制点
    drawPoints(curve.controlPoints(), 0.2f, 0.4f, 0.8f, 12.0f);
    
    // 5. 高亮当前 u 对应的曲线点
    if (u >= 0.0 && u <= 1.0) 
    {
        nurbs::PointList highlight = {curve.evaluate(u)};
        drawPoints(highlight, 1.0f, 0.2f, 0.2f, 16.0f);
    }
}

void CurveRenderer::drawCurve(const nurbs::PointList& points, float r, float g, float b, float lineWidth) 
{
    if (points.size() < 2) 
        return;
    
    // 准备顶点数据
    std::vector<float> vertices;
    vertices.reserve(points.size() * 3);
    for (const auto& p : points) 
    {
        vertices.push_back(static_cast<float>(p.x));
        vertices.push_back(static_cast<float>(p.y));
        vertices.push_back(static_cast<float>(p.z));
    }
    
    // 上传数据
    glBindVertexArray(m_lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_lineVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(float), vertices.data());
    
    // 设置颜色并绘制
    m_shader->setVec4("color", r, g, b, 1.0f);
    glLineWidth(lineWidth);
    glDrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(points.size()));
    
    glBindVertexArray(0);
}

void CurveRenderer::drawControlPolygon(const nurbs::PointList& points, float r, float g, float b) 
{
    drawCurve(points, r, g, b, 1.0f);
}

void CurveRenderer::drawPoints(const nurbs::PointList& points, float r, float g, float b, float size) 
{
    if (points.empty()) 
        return;
    
    // 准备顶点数据
    std::vector<float> vertices;
    vertices.reserve(points.size() * 3);
    for (const auto& p : points) 
    {
        vertices.push_back(static_cast<float>(p.x));
        vertices.push_back(static_cast<float>(p.y));
        vertices.push_back(static_cast<float>(p.z));
    }
    
    // 上传数据
    glBindVertexArray(m_pointVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_pointVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(float), vertices.data());
    
    // 设置颜色并绘制
    m_shader->setVec4("color", r, g, b, 1.0f);
    glPointSize(size);
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(points.size()));
    glBindVertexArray(0);
}

void CurveRenderer::drawDeCasteljauPyramid(const std::vector<nurbs::PointList>& pyramid) 
{
    // 从第1层开始（第0层是原始控制点，已单独绘制）
    for (size_t r = 1; r < pyramid.size(); r++) 
    {
        int colorIdx = r % NUM_COLORS;
        float cr = LAYER_COLORS[colorIdx][0];
        float cg = LAYER_COLORS[colorIdx][1];
        float cb = LAYER_COLORS[colorIdx][2];
        
        // 绘制这一层的连线
        if (pyramid[r].size() >= 2)
            drawCurve(pyramid[r], cr, cg, cb, 1.5f);
        
        // 绘制这一层的点
        drawPoints(pyramid[r], cr, cg, cb, 8.0f);
    }
}

} // namespace viewer
