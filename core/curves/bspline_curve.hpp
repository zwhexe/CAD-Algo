#pragma once

#include "core/math/point.hpp"
#include <vector>
#include <stdexcept>

namespace nurbs 
{

/**
 * B-spline 曲线类
 * 
 * 基于 The NURBS Book 第2章实现
 * 
 * B-spline 曲线定义:
 *   C(u) = Σ N_{i,p}(u) * P_i
 * 
 * 其中:
 *   - P_i 是控制点
 *   - N_{i,p}(u) 是 p 次 B-spline 基函数
 *   - 参数 u ∈ [u_p, u_{m-p}]（有效参数范围）
 */
class BSplineCurve 

{
public:
    /**
     * 构造函数
     * 
     * @param controlPoints 控制点列表 (n+1 个点)
     * @param degree 曲线次数 p
     * @param knots 节点向量 (m+1 个节点，其中 m = n + p + 1)
     */
    BSplineCurve(const PointList& controlPoints, int degree, const std::vector<double>& knots);
    
    /**
     * 使用均匀节点向量构造
     * 自动生成 clamped uniform knot vector
     */
    BSplineCurve(const PointList& controlPoints, int degree);
    
    /**
     * 计算曲线上的点 - de Boor 算法
     * The NURBS Book Algorithm A3.1 (Page 82)
     * 
     * @param u 参数值
     * @return 曲线上的点 C(u)
     */
    Point evaluate(double u) const;
    
    /**
     * 计算 B-spline 基函数值
     * The NURBS Book Algorithm A2.2 (Page 70)
     * 
     * @param i 基函数索引
     * @param p 基函数次数
     * @param u 参数值
     * @return N_{i,p}(u)
     */
    double basisFunction(int i, int p, double u) const;
    
    /**
     * 计算所有非零基函数
     * The NURBS Book Algorithm A2.4 (Page 74)
     * 
     * @param span 节点区间索引
     * @param u 参数值
     * @return p+1 个非零基函数值 [N_{span-p,p}, ..., N_{span,p}]
     */
    std::vector<double> basisFunctions(int span, double u) const;
    
    /**
     * 查找参数 u 所在的节点区间
     * The NURBS Book Algorithm A2.1 (Page 68)
     * 
     * @param u 参数值
     * @return 节点区间索引 i，使得 u ∈ [u_i, u_{i+1})
     */
    int findSpan(double u) const;
    
    /**
     * 节点插入
     * The NURBS Book Algorithm A5.1 (Page 151)
     * 
     * 在参数 u 处插入节点，不改变曲线形状
     * 
     * @param u 插入位置
     * @return 新的 B-spline 曲线
     */
    BSplineCurve knotInsert(double u) const;
    
    /**
     * de Boor 算法的完整金字塔（用于可视化）
     */
    std::vector<PointList> deBoorPyramid(double u) const;
    
    /**
     * 生成采样点
     */
    PointList sample(int numSamples = 100) const;
    
    // Getters
    const PointList& controlPoints() const { return m_controlPoints; }
    const std::vector<double>& knots() const { return m_knots; }
    int degree() const { return m_degree; }
    int numControlPoints() const { return static_cast<int>(m_controlPoints.size()); }
    int numKnots() const { return static_cast<int>(m_knots.size()); }
    
    // 有效参数范围
    double uMin() const { return m_knots[m_degree]; }
    double uMax() const { return m_knots[m_knots.size() - 1 - m_degree]; }
    
    // Setters
    void setControlPoint(int index, const Point& p);

private:
    PointList m_controlPoints;    // n+1 个控制点
    std::vector<double> m_knots;  // m+1 个节点
    int m_degree;                 // 次数 p
    
    void validateKnotVector() const;
    static std::vector<double> generateClampedUniformKnots(int n, int p);
};

/**
 * 创建常用的 B-spline 曲线
 */
namespace BSplineFactory 
{
    /**
     * 创建均匀 B-spline 曲线
     */
    BSplineCurve createUniform(const PointList& controlPoints, int degree);
    
    /**
     * 创建插值 B-spline（经过所有给定点）
     * The NURBS Book Algorithm A9.1
     */
    BSplineCurve createInterpolating(const PointList& throughPoints, int degree);
}

} // namespace nurbs
