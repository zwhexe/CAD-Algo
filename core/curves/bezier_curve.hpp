#pragma once

#include "core/math/point.hpp"
#include <vector>
#include <stdexcept>

namespace nurbs 
{

//==============================================================================
// Section 1.3: Bézier Curves (非有理)
//==============================================================================

/**
 * Bézier 曲线类
 * 
 * The NURBS Book Section 1.3
 * 
 * 定义 (Equation 1.7):
 *   C(u) = Σ_{i=0}^{n} B_{i,n}(u) · P_i,  u ∈ [0,1]
 * 
 * 其中 Bernstein 基函数 (Equation 1.8):
 *   B_{i,n}(u) = C(n,i) · u^i · (1-u)^{n-i}
 * 
 * 性质:
 *   - 端点插值: C(0) = P_0, C(1) = P_n
 *   - 仿射不变性
 *   - 凸包性质
 *   - 变差缩减性
 */
class BezierCurve 

{
public:
    explicit BezierCurve(const PointList& controlPoints);
    
    //--------------------------------------------------------------------------
    // Algorithm A1.1: de Casteljau 算法 (Page 24)
    //--------------------------------------------------------------------------
    /**
     * de Casteljau 递推公式:
     *   P_i^{[0]} = P_i                    (初始条件)
     *   P_i^{[r]} = (1-u)·P_i^{[r-1]} + u·P_{i+1}^{[r-1]}  (r=1,...,n)
     * 
     * 最终结果: C(u) = P_0^{[n]}
     */
    Point evaluate(double u) const;
    
    /**
     * 返回 de Casteljau 金字塔的所有中间点
     * pyramid[r][i] = P_i^{[r]}
     * 
     * 用于:
     * 1. 可视化算法过程
     * 2. 曲线细分 (subdivision)
     * 3. 求导数
     */
    std::vector<PointList> deCasteljauPyramid(double u) const;
    
    //--------------------------------------------------------------------------
    // 直接使用 Bernstein 多项式求值 (用于对比)
    //--------------------------------------------------------------------------
    Point evaluateBernstein(double u) const;
    
    //--------------------------------------------------------------------------
    // Section 1.6: 曲线细分 (Subdivision)
    //--------------------------------------------------------------------------
    /**
     * 在参数 u 处将曲线分成两段
     * 
     * 利用 de Casteljau 金字塔:
     *   左曲线控制点: P_0^{[0]}, P_0^{[1]}, ..., P_0^{[n]} (左边界)
     *   右曲线控制点: P_0^{[n]}, P_1^{[n-1]}, ..., P_n^{[0]} (右边界，反向)
     */
    std::pair<PointList, PointList> subdivide(double u) const;
    
    //--------------------------------------------------------------------------
    // Section 1.3: 导数 (Derivatives)
    //--------------------------------------------------------------------------
    /**
     * 一阶导数 (Equation 1.12):
     *   C'(u) = n · Σ_{i=0}^{n-1} B_{i,n-1}(u) · (P_{i+1} - P_i)
     * 
     * 即: 导数曲线是 n-1 次 Bézier 曲线，控制点为 n·(P_{i+1} - P_i)
     */
    Point derivative(double u, int order = 1) const;
    
    /**
     * 获取导数曲线的控制点 (hodograph)
     */
    PointList derivativeControlPoints(int order = 1) const;
    
    //--------------------------------------------------------------------------
    // Section 1.7: Degree Elevation (升阶)
    //--------------------------------------------------------------------------
    /**
     * 将 n 次曲线提升为 n+1 次，形状不变
     * 
     * 新控制点 (Equation 1.17):
     *   P_i^{new} = (i/(n+1))·P_{i-1} + (1 - i/(n+1))·P_i
     */
    BezierCurve degreeElevate() const;
    
    //--------------------------------------------------------------------------
    // 辅助函数
    //--------------------------------------------------------------------------
    PointList sample(int numSamples = 100) const;
    
    const PointList& controlPoints() const { return m_controlPoints; }
    int degree() const { return static_cast<int>(m_controlPoints.size()) - 1; }
    
    void setControlPoint(int index, const Point& p);
    void setControlPoints(const PointList& points);

    static double binomial(int n, int k);
    static double bernsteinBasis(int i, int n, double u);

private:
    PointList m_controlPoints;
};


//==============================================================================
// Section 1.4: Rational Bézier Curves (有理 Bézier 曲线)
//==============================================================================

/**
 * 有理 Bézier 曲线
 * 
 * The NURBS Book Section 1.4
 * 
 * 定义 (Equation 1.18):
 *   C(u) = Σ_{i=0}^{n} R_{i,n}(u) · P_i
 * 
 * 有理基函数 (Equation 1.19):
 *   R_{i,n}(u) = (B_{i,n}(u) · w_i) / (Σ_{j=0}^{n} B_{j,n}(u) · w_j)
 * 
 * 重要性质:
 *   - 可以精确表示圆锥曲线 (圆、椭圆、抛物线、双曲线)
 *   - 当所有 w_i = 1 时退化为普通 Bézier 曲线
 *   - 射影不变性 (projective invariance)
 * 
 * 算法思路:
 *   使用齐次坐标，将有理曲线转化为高一维空间中的非有理曲线
 *   C^w(u) = Σ B_{i,n}(u) · (w_i·P_i, w_i)  在 4D 空间
 *   然后透视除法回到 3D
 */
class RationalBezierCurve 

{
public:
    /**
     * 构造函数
     * @param controlPoints 控制点
     * @param weights 权重 (与控制点一一对应)
     */
    RationalBezierCurve(const PointList& controlPoints, const std::vector<double>& weights);
    
    /**
     * 从普通 Bézier 构造 (所有权重为 1)
     */
    explicit RationalBezierCurve(const PointList& controlPoints);
    
    //--------------------------------------------------------------------------
    // Algorithm A1.2: 有理 de Casteljau 算法 (Page 28)
    //--------------------------------------------------------------------------
    /**
     * 在齐次坐标空间应用 de Casteljau，然后投影
     * 
     * 步骤:
     * 1. 将 (P_i, w_i) 转换为齐次坐标 P_i^w = (w_i·x, w_i·y, w_i·z, w_i)
     * 2. 在 4D 空间做 de Casteljau
     * 3. 透视除法: 将 (X, Y, Z, W) 转换为 (X/W, Y/W, Z/W)
     */
    Point evaluate(double u) const;
    
    /**
     * 有理基函数 R_{i,n}(u)
     */
    double rationalBasis(int i, double u) const;
    
    /**
     * 返回有理 de Casteljau 金字塔 (投影后的 3D 点)
     */
    std::vector<PointList> deCasteljauPyramid(double u) const;
    
    //--------------------------------------------------------------------------
    // 导数 (Section 1.4.2)
    //--------------------------------------------------------------------------
    /**
     * 有理曲线导数 (更复杂，涉及商的求导)
     * 
     * Equation 1.24:
     *   C'(u) = (A'(u)·w(u) - A(u)·w'(u)) / w(u)²
     * 
     * 其中 A(u) = Σ B_{i,n}(u)·w_i·P_i, w(u) = Σ B_{i,n}(u)·w_i
     */
    Point derivative(double u, int order = 1) const;
    
    //--------------------------------------------------------------------------
    // 细分
    //--------------------------------------------------------------------------
    /**
     * 在齐次空间细分，然后转换回来
     */
    std::pair<RationalBezierCurve, RationalBezierCurve> subdivide(double u) const;
    
    //--------------------------------------------------------------------------
    // 辅助函数
    //--------------------------------------------------------------------------
    PointList sample(int numSamples = 100) const;
    
    const PointList& controlPoints() const { return m_controlPoints; }
    const std::vector<double>& weights() const { return m_weights; }
    int degree() const { return static_cast<int>(m_controlPoints.size()) - 1; }
    
    void setControlPoint(int index, const Point& p);
    void setWeight(int index, double w);

private:
    PointList m_controlPoints;
    std::vector<double> m_weights;
    
    // 齐次坐标表示 (w*x, w*y, w*z, w)
    struct HomogeneousPoint 
    {
        double x, y, z, w;
        
        HomogeneousPoint(double x = 0, double y = 0, double z = 0, double w = 1)
            : x(x), y(y), z(z), w(w) {}
        
        // 从 3D 点 + 权重构造
        HomogeneousPoint(const Point& p, double weight)
            : x(p.x * weight), y(p.y * weight), z(p.z * weight), w(weight) {}
        
        // 线性插值
        static HomogeneousPoint lerp(const HomogeneousPoint& a, 
                                     const HomogeneousPoint& b, double t) 
        {
            return { (1-t) * a.x + t * b.x,
                     (1-t) * a.y + t * b.y,
                     (1-t) * a.z + t * b.z,
                     (1-t) * a.w + t * b.w };
        }
        
        // 透视除法，投影回 3D
        Point project() const 
        {
            if (std::abs(w) < 1e-10) 
                return Point(0, 0, 0);
            return Point(x/w, y/w, z/w);
        }
        
        double weight() const { return w; }
    };
    
    std::vector<HomogeneousPoint> toHomogeneous() const;
};


//==============================================================================
// Section 1.4.3: 圆锥曲线 (Conic Sections)
//==============================================================================

namespace ConicFactory 
{
    /**
     * 创建圆弧 (二次有理 Bézier)
     * 
     * The NURBS Book Section 1.4.3, Example 1.5
     * 
     * 圆弧由 3 个控制点和 1 个中间权重定义:
     *   P0 = 起点 (w=1)
     *   P1 = 切线交点 (w = cos(θ/2), θ 为弧度角)
     *   P2 = 终点 (w=1)
     */
    RationalBezierCurve createArc(const Point& center, double radius,
                                   double startAngle, double endAngle);
    
    /**
     * 创建 90° 圆弧 (最常用)
     */
    RationalBezierCurve createQuarterCircle(const Point& center, double radius,
                                             int quadrant = 0);
    
    /**
     * 创建完整圆 (4 段 90° 圆弧)
     */
    std::vector<RationalBezierCurve> createFullCircle(const Point& center, double radius);
    
    /**
     * 创建椭圆弧
     */
    RationalBezierCurve createEllipticArc(const Point& center,
                                           double semiMajor, double semiMinor,
                                           double startAngle, double endAngle);
}

} // namespace nurbs
