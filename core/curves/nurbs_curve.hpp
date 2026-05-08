#pragma once

#include "core/math/point.hpp"
#include "core/curves/bspline_curve.hpp"
#include <vector>

namespace nurbs 
{

/**
 * 带权重的控制点（齐次坐标）
 */
struct WeightedPoint 

{
    Point point;
    double weight;
    
    WeightedPoint(const Point& p = Point(), double w = 1.0)
        : point(p), weight(w) {}
    
    // 转换为齐次坐标 (wx, wy, wz, w)
    Point toHomogeneous() const {
        return Point(point.x * weight, point.y * weight, point.z * weight);
    }
    
    // 从齐次坐标恢复
    static WeightedPoint fromHomogeneous(const Point& h, double w) {
        if (std::abs(w) < 1e-10) {
            return WeightedPoint(Point(0, 0, 0), 0);
        }
        return WeightedPoint(Point(h.x / w, h.y / w, h.z / w), w);
    }
};

using WeightedPointList = std::vector<WeightedPoint>;

/**
 * NURBS 曲线类
 * 
 * 基于 The NURBS Book 第4章实现
 * 
 * NURBS = Non-Uniform Rational B-Spline
 * 
 * 定义:
 *   C(u) = Σ R_{i,p}(u) * P_i
 * 
 * 其中有理基函数:
 *   R_{i,p}(u) = (N_{i,p}(u) * w_i) / Σ (N_{j,p}(u) * w_j)
 * 
 * NURBS 的优势:
 * 1. 可以精确表示圆锥曲线（圆、椭圆、抛物线、双曲线）
 * 2. 权重提供额外的形状控制
 * 3. 向 B-spline 的退化：当所有权重相等时，退化为 B-spline
 */
class NurbsCurve 

{
public:
    /**
     * 构造函数
     * 
     * @param controlPoints 带权重的控制点
     * @param degree 曲线次数
     * @param knots 节点向量
     */
    NurbsCurve(const WeightedPointList& controlPoints, int degree, 
               const std::vector<double>& knots);
    
    /**
     * 使用均匀节点向量构造
     */
    NurbsCurve(const WeightedPointList& controlPoints, int degree);
    
    /**
     * 从普通 B-spline 构造（所有权重设为 1）
     */
    static NurbsCurve fromBSpline(const BSplineCurve& bspline);
    
    /**
     * 计算曲线上的点
     * The NURBS Book Algorithm A4.1 (Page 124)
     * 
     * 使用齐次坐标在 4D 空间做 B-spline 求值，
     * 然后透视除法回到 3D
     */
    Point evaluate(double u) const;
    
    /**
     * 计算有理基函数值
     * R_{i,p}(u)
     */
    double rationalBasis(int i, double u) const;
    
    /**
     * 节点插入（保持曲线形状不变）
     */
    NurbsCurve knotInsert(double u) const;
    
    /**
     * 生成采样点
     */
    PointList sample(int numSamples = 100) const;
    
    // Getters
    const WeightedPointList& controlPoints() const { return m_controlPoints; }
    PointList controlPointsWithoutWeights() const;
    std::vector<double> weights() const;
    const std::vector<double>& knots() const { return m_knots; }
    int degree() const { return m_degree; }
    
    double uMin() const { return m_knots[m_degree]; }
    double uMax() const { return m_knots[m_knots.size() - 1 - m_degree]; }
    
    // Setters
    void setControlPoint(int index, const Point& p);
    void setWeight(int index, double w);
    void setControlPointAndWeight(int index, const Point& p, double w);

private:
    WeightedPointList m_controlPoints;
    std::vector<double> m_knots;
    int m_degree;
    
    // 内部使用的 B-spline（在齐次坐标空间）
    mutable std::unique_ptr<BSplineCurve> m_homogeneousCurve;
    void updateHomogeneousCurve() const;
};

/**
 * 常用 NURBS 曲线工厂
 */
namespace NurbsFactory 
{
    /**
     * 创建圆弧
     * The NURBS Book Example 7.1 (Page 308)
     * 
     * @param center 圆心
     * @param radius 半径
     * @param startAngle 起始角度（弧度）
     * @param endAngle 结束角度（弧度）
     * @param normal 法向量（默认 z 轴）
     */
    NurbsCurve createArc(const Point& center, double radius,
                         double startAngle, double endAngle,
                         const Point& normal = Point(0, 0, 1));
    
    /**
     * 创建完整圆
     */
    NurbsCurve createCircle(const Point& center, double radius,
                            const Point& normal = Point(0, 0, 1));
    
    /**
     * 创建椭圆
     */
    NurbsCurve createEllipse(const Point& center, 
                             double semiMajor, double semiMinor,
                             const Point& normal = Point(0, 0, 1));
}

} // namespace nurbs
