#pragma once

#include "core/math/point.hpp"
#include "core/curves/bezier_curve.hpp"
#include <vector>
#include <stdexcept>

namespace nurbs 
{

//==============================================================================
// Section 1.5: Tensor Product Bézier Surfaces
//==============================================================================

/**
 * 控制点网格 (二维数组)
 * 
 * 索引约定: grid[i][j]
 *   i: u 方向 (0 到 n)
 *   j: v 方向 (0 到 m)
 */
using ControlPointGrid = std::vector<std::vector<Point>>;

/**
 * 曲面上的点 + 导数信息
 */
struct SurfacePoint 

{
    Point position;      // S(u,v)
    Point derivativeU;   // ∂S/∂u
    Point derivativeV;   // ∂S/∂v
    Point normal;        // 法向量 (derivativeU × derivativeV)
};


/**
 * Tensor Product Bézier 曲面
 * 
 * The NURBS Book Section 1.5
 * 
 * 定义 (Equation 1.25):
 *   S(u,v) = Σ_{i=0}^{n} Σ_{j=0}^{m} B_{i,n}(u) · B_{j,m}(v) · P_{i,j}
 * 
 * 其中:
 *   - P_{i,j} 是 (n+1) × (m+1) 的控制点网格
 *   - B_{i,n}(u), B_{j,m}(v) 是 Bernstein 基函数
 *   - u, v ∈ [0, 1]
 * 
 * 关键性质:
 *   - 四角插值: S(0,0)=P_{0,0}, S(1,0)=P_{n,0}, S(0,1)=P_{0,m}, S(1,1)=P_{n,m}
 *   - 边界曲线是 Bézier 曲线
 *   - 仿射不变性
 *   - 凸包性质
 * 
 * 几何直觉:
 *   "曲线的曲线" - 可以看作 Bézier 曲线沿另一条 Bézier 曲线移动形成的轨迹
 */
class BezierSurface 

{
public:
    /**
     * 构造函数
     * @param controlPoints (n+1) × (m+1) 控制点网格
     *                      controlPoints[i][j] 对应 P_{i,j}
     */
    explicit BezierSurface(const ControlPointGrid& controlPoints);
    
    /**
     * 便捷构造: 指定 u 方向和 v 方向的次数
     */
    BezierSurface(int degreeU, int degreeV);
    
    //--------------------------------------------------------------------------
    // Algorithm A1.3: de Casteljau 曲面求值 (Page 32)
    //--------------------------------------------------------------------------
    /**
     * 使用 de Casteljau 算法求值
     * 
     * 两步法:
     * 1. 固定 u，对每一行做 de Casteljau，得到 m+1 个点 Q_j(u)
     * 2. 对 Q_j(u) 再做一次 de Casteljau (关于 v)
     * 
     * 或者反过来: 先固定 v，再处理 u (结果相同)
     */
    Point evaluate(double u, double v) const;
    
    /**
     * 返回完整的曲面信息 (位置 + 导数 + 法向量)
     */
    SurfacePoint evaluateFull(double u, double v) const;
    
    //--------------------------------------------------------------------------
    // 等参线 (Isoparametric Curves)
    //--------------------------------------------------------------------------
    /**
     * 获取固定 u 时的等参线 (v 方向的 Bézier 曲线)
     * 
     * 等参线 C_u(v) = S(u, v)，u 固定
     * 这是一条 m 次 Bézier 曲线
     */
    BezierCurve isoCurveU(double u) const;
    
    /**
     * 获取固定 v 时的等参线 (u 方向的 Bézier 曲线)
     * 
     * 等参线 C_v(u) = S(u, v)，v 固定
     * 这是一条 n 次 Bézier 曲线
     */
    BezierCurve isoCurveV(double v) const;
    
    //--------------------------------------------------------------------------
    // 边界曲线
    //--------------------------------------------------------------------------
    /**
     * 四条边界曲线
     * 
     * bottom: S(u, 0), u ∈ [0,1]  - 控制点 P_{i,0}
     * top:    S(u, 1), u ∈ [0,1]  - 控制点 P_{i,m}
     * left:   S(0, v), v ∈ [0,1]  - 控制点 P_{0,j}
     * right:  S(1, v), v ∈ [0,1]  - 控制点 P_{n,j}
     */
    BezierCurve boundaryBottom() const;
    BezierCurve boundaryTop() const;
    BezierCurve boundaryLeft() const;
    BezierCurve boundaryRight() const;
    
    //--------------------------------------------------------------------------
    // 导数 (Partial Derivatives)
    //--------------------------------------------------------------------------
    /**
     * 偏导数 ∂S/∂u
     * 
     * Equation 1.27:
     *   ∂S/∂u = n · Σ_i Σ_j B_{i,n-1}(u) · B_{j,m}(v) · (P_{i+1,j} - P_{i,j})
     */
    Point derivativeU(double u, double v) const;
    
    /**
     * 偏导数 ∂S/∂v
     */
    Point derivativeV(double u, double v) const;
    
    /**
     * 法向量 (未归一化)
     * N = ∂S/∂u × ∂S/∂v
     */
    Point normal(double u, double v) const;
    
    //--------------------------------------------------------------------------
    // 曲面细分
    //--------------------------------------------------------------------------
    /**
     * 在 u 方向细分
     * @param u 细分位置
     * @return pair<左曲面, 右曲面>
     */
    std::pair<BezierSurface, BezierSurface> subdivideU(double u) const;
    
    /**
     * 在 v 方向细分
     */
    std::pair<BezierSurface, BezierSurface> subdivideV(double v) const;
    
    /**
     * 同时在两个方向细分，得到 4 个子曲面
     */
    std::vector<BezierSurface> subdivide(double u, double v) const;
    
    //--------------------------------------------------------------------------
    // 升阶
    //--------------------------------------------------------------------------
    /**
     * u 方向升阶
     */
    BezierSurface degreeElevateU() const;
    
    /**
     * v 方向升阶
     */
    BezierSurface degreeElevateV() const;
    
    //--------------------------------------------------------------------------
    // 采样和网格生成
    //--------------------------------------------------------------------------
    /**
     * 生成采样网格
     * @param samplesU u 方向采样数
     * @param samplesV v 方向采样数
     * @return 采样点网格
     */
    ControlPointGrid sampleGrid(int samplesU, int samplesV) const;
    
    /**
     * 生成三角形网格 (用于渲染)
     * @return pair<顶点列表, 三角形索引>
     */
    std::pair<std::vector<Point>, std::vector<std::array<int, 3>>> 
        triangulate(int samplesU, int samplesV) const;
    
    //--------------------------------------------------------------------------
    // 辅助函数
    //--------------------------------------------------------------------------
    const ControlPointGrid& controlPoints() const { return m_controlPoints; }
    int degreeU() const { return static_cast<int>(m_controlPoints.size()) - 1; }
    int degreeV() const { return m_controlPoints.empty() ? 0 : 
                                 static_cast<int>(m_controlPoints[0].size()) - 1; }
    
    Point& controlPoint(int i, int j) { return m_controlPoints[i][j]; }
    const Point& controlPoint(int i, int j) const { return m_controlPoints[i][j]; }
    
    void setControlPoint(int i, int j, const Point& p);

private:
    ControlPointGrid m_controlPoints;  // (n+1) × (m+1) 网格
    
    void validateGrid() const;
};


//==============================================================================
// Section 1.5.1: Rational Tensor Product Bézier Surfaces
//==============================================================================

/**
 * 带权重的控制点网格
 */
struct WeightedControlPoint 

{
    Point point;
    double weight;
    
    WeightedControlPoint(const Point& p = Point(), double w = 1.0)
        : point(p), weight(w) {}
};

using WeightedControlGrid = std::vector<std::vector<WeightedControlPoint>>;

/**
 * 有理 Tensor Product Bézier 曲面
 * 
 * The NURBS Book Section 1.5.1
 * 
 * 定义:
 *   S(u,v) = Σ_i Σ_j R_{i,j}(u,v) · P_{i,j}
 * 
 * 有理基函数:
 *   R_{i,j}(u,v) = (B_{i,n}(u) · B_{j,m}(v) · w_{i,j}) / 
 *                  (Σ_k Σ_l B_{k,n}(u) · B_{l,m}(v) · w_{k,l})
 * 
 * 可以精确表示:
 *   - 球面、圆柱面、圆锥面
 *   - 圆环面 (torus)
 *   - 其他二次曲面
 */
class RationalBezierSurface 

{
public:
    RationalBezierSurface(const ControlPointGrid& controlPoints,
                          const std::vector<std::vector<double>>& weights);
    
    explicit RationalBezierSurface(const WeightedControlGrid& weightedPoints);
    
    // 从非有理曲面构造
    explicit RationalBezierSurface(const BezierSurface& surface);
    
    //--------------------------------------------------------------------------
    // Algorithm A1.4: 有理曲面求值 (Page 34)
    //--------------------------------------------------------------------------
    Point evaluate(double u, double v) const;
    
    /**
     * 完整求值 (带导数和法向量)
     */
    SurfacePoint evaluateFull(double u, double v) const;
    
    //--------------------------------------------------------------------------
    // 辅助函数
    //--------------------------------------------------------------------------
    ControlPointGrid controlPointGrid() const;
    std::vector<std::vector<double>> weightGrid() const;
    
    int degreeU() const { return static_cast<int>(m_weightedPoints.size()) - 1; }
    int degreeV() const { return m_weightedPoints.empty() ? 0 :
                                 static_cast<int>(m_weightedPoints[0].size()) - 1; }
    
    void setControlPoint(int i, int j, const Point& p);
    void setWeight(int i, int j, double w);

private:
    WeightedControlGrid m_weightedPoints;
    
    // 齐次坐标表示
    struct HomogeneousPoint4D 

{
        double x, y, z, w;
        
        HomogeneousPoint4D(double x = 0, double y = 0, double z = 0, double w = 1)
            : x(x), y(y), z(z), w(w) {}
        
        HomogeneousPoint4D(const Point& p, double weight)
            : x(p.x * weight), y(p.y * weight), z(p.z * weight), w(weight) {}
        
        static HomogeneousPoint4D lerp(const HomogeneousPoint4D& a,
                                        const HomogeneousPoint4D& b, double t) {
            return {
                (1-t)*a.x + t*b.x,
                (1-t)*a.y + t*b.y,
                (1-t)*a.z + t*b.z,
                (1-t)*a.w + t*b.w
            };
        }
        
        Point project() const {
            if (std::abs(w) < 1e-10) return Point(0, 0, 0);
            return Point(x/w, y/w, z/w);
        }
    };
    
    std::vector<std::vector<HomogeneousPoint4D>> toHomogeneous() const;
};


//==============================================================================
// 工厂函数: 创建常用曲面
//==============================================================================

namespace SurfaceFactory 
{
    /**
     * 创建双线性曲面 (n=m=1)
     * 最简单的 Bézier 曲面，由四个角点定义
     */
    BezierSurface createBilinear(const Point& p00, const Point& p10,
                                  const Point& p01, const Point& p11);
    
    /**
     * 创建平面
     */
    BezierSurface createPlane(const Point& origin, 
                               const Point& axisU, const Point& axisV,
                               double sizeU = 1.0, double sizeV = 1.0);
    
    /**
     * 创建圆柱面片
     */
    RationalBezierSurface createCylinderPatch(const Point& center,
                                               double radius, double height,
                                               double startAngle, double endAngle);
    
    /**
     * 创建球面片 (八分之一球)
     */
    RationalBezierSurface createSpherePatch(const Point& center, double radius);
    
    /**
     * 创建圆锥面片
     */
    RationalBezierSurface createConePatch(const Point& apex, const Point& baseCenter,
                                           double baseRadius, double startAngle, double endAngle);
    
    /**
     * 创建旋转曲面
     * @param profile 轮廓曲线 (在 xz 平面)
     * @param axis 旋转轴 (默认 z 轴)
     * @param startAngle, endAngle 旋转角度范围
     */
    RationalBezierSurface createRevolution(const RationalBezierCurve& profile,
                                            const Point& axis = Point(0, 0, 1),
                                            double startAngle = 0,
                                            double endAngle = M_PI / 2);
}

} // namespace nurbs
