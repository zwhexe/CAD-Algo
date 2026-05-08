#include "core/curves/nurbs_curve.hpp"
#include <cmath>
#include <algorithm>

namespace nurbs 
{

NurbsCurve::NurbsCurve(const WeightedPointList& controlPoints, int degree,
                       const std::vector<double>& knots)
    : m_controlPoints(controlPoints)
    , m_knots(knots)
    , m_degree(degree)
{
    if (controlPoints.size() < 2) {
        throw std::invalid_argument("NURBS curve requires at least 2 control points");
    }
    updateHomogeneousCurve();
}

NurbsCurve::NurbsCurve(const WeightedPointList& controlPoints, int degree)
    : m_controlPoints(controlPoints)
    , m_degree(degree)
{
    if (controlPoints.size() < 2) {
        throw std::invalid_argument("NURBS curve requires at least 2 control points");
    }
    
    // 生成 clamped uniform 节点向量
    int n = static_cast<int>(controlPoints.size()) - 1;
    int m = n + degree + 1;
    m_knots.resize(m + 1);
    
    for (int i = 0; i <= degree; i++) {
        m_knots[i] = 0.0;
    }
    
    int numInternalKnots = m - 2 * degree - 1;
    for (int i = 1; i <= numInternalKnots; i++) {
        m_knots[degree + i] = static_cast<double>(i) / (numInternalKnots + 1);
    }
    
    for (int i = m - degree; i <= m; i++) {
        m_knots[i] = 1.0;
    }
    
    updateHomogeneousCurve();
}

NurbsCurve NurbsCurve::fromBSpline(const BSplineCurve& bspline) {
    WeightedPointList wpts;
    for (const auto& p : bspline.controlPoints()) {
        wpts.emplace_back(p, 1.0);
    }
    return NurbsCurve(wpts, bspline.degree(), bspline.knots());
}

void NurbsCurve::updateHomogeneousCurve() const {
    // 将带权控制点转换为齐次坐标形式
    // 4D 点: (w*x, w*y, w*z) - 我们用 Point 的 x,y,z 存储
    // 权重单独存储在另一个 channel
    
    PointList homogeneousPoints;
    homogeneousPoints.reserve(m_controlPoints.size());
    
    for (const auto& wp : m_controlPoints) {
        homogeneousPoints.push_back(wp.toHomogeneous());
    }
    
    m_homogeneousCurve = std::make_unique<BSplineCurve>(
        homogeneousPoints, m_degree, m_knots);
}

/**
 * Algorithm A4.1 - NURBS 曲线求值
 * 
 * 步骤：
 * 1. 在齐次坐标空间做 B-spline 求值
 * 2. 透视除法：将 4D 齐次坐标转回 3D
 */
Point NurbsCurve::evaluate(double u) const {
    u = std::max(uMin(), std::min(uMax(), u));
    
    if (!m_homogeneousCurve) 
{
        updateHomogeneousCurve();
    }
    
    // 在齐次坐标空间求值
    Point Cw = m_homogeneousCurve->evaluate(u);
    
    // 计算权重和
    int span = m_homogeneousCurve->findSpan(u);
    auto N = m_homogeneousCurve->basisFunctions(span, u);
    
    double w = 0.0;
    for (int i = 0; i <= m_degree; i++) {
        w += N[i] * m_controlPoints[span - m_degree + i].weight;
    }
    
    // 透视除法
    if (std::abs(w) < 1e-10) {
        return Point(0, 0, 0);
    }
    
    return Point(Cw.x / w, Cw.y / w, Cw.z / w);
}

double NurbsCurve::rationalBasis(int i, double u) const {
    if (!m_homogeneousCurve) 
{
        updateHomogeneousCurve();
    }
    
    // N_{i,p}(u) * w_i
    double Nw = m_homogeneousCurve->basisFunction(i, m_degree, u) * 
                m_controlPoints[i].weight;
    
    // 分母：所有 N_{j,p}(u) * w_j 的和
    double denom = 0.0;
    for (size_t j = 0; j < m_controlPoints.size(); j++) {
        denom += m_homogeneousCurve->basisFunction(j, m_degree, u) * 
                 m_controlPoints[j].weight;
    }
    
    if (std::abs(denom) < 1e-10) {
        return 0.0;
    }
    
    return Nw / denom;
}

NurbsCurve NurbsCurve::knotInsert(double u) const {
    if (!m_homogeneousCurve) 
{
        updateHomogeneousCurve();
    }
    
    // 在齐次空间做节点插入
    auto newHomoCurve = m_homogeneousCurve->knotInsert(u);
    
    // 计算新的权重
    int k = m_homogeneousCurve->findSpan(u);
    int n = static_cast<int>(m_controlPoints.size()) - 1;
    
    WeightedPointList newPoints(n + 2);
    
    // 前 k-p+1 个点不变
    for (int i = 0; i <= k - m_degree; i++) {
        newPoints[i] = m_controlPoints[i];
    }
    
    // 后 n-k 个点不变
    for (int i = k; i <= n; i++) {
        newPoints[i + 1] = m_controlPoints[i];
    }
    
    // 中间的点需要重新计算
    for (int i = k - m_degree + 1; i <= k; i++) {
        double alpha = (u - m_knots[i]) / (m_knots[i + m_degree] - m_knots[i]);
        
        // 权重的线性插值
        double newWeight = (1.0 - alpha) * m_controlPoints[i - 1].weight +
                          alpha * m_controlPoints[i].weight;
        
        // 齐次坐标的线性插值
        Point h1 = m_controlPoints[i - 1].toHomogeneous();
        Point h2 = m_controlPoints[i].toHomogeneous();
        Point newH = h1 * (1.0 - alpha) + h2 * alpha;
        
        newPoints[i] = WeightedPoint::fromHomogeneous(newH, newWeight);
    }
    
    return NurbsCurve(newPoints, m_degree, newHomoCurve.knots());
}

PointList NurbsCurve::sample(int numSamples) const {
    PointList points;
    points.reserve(numSamples);
    
    double uStart = uMin();
    double uEnd = uMax();
    
    for (int i = 0; i < numSamples; i++) {
        double t = static_cast<double>(i) / (numSamples - 1);
        double u = uStart + t * (uEnd - uStart);
        points.push_back(evaluate(u));
    }
    
    return points;
}

PointList NurbsCurve::controlPointsWithoutWeights() const {
    PointList points;
    points.reserve(m_controlPoints.size());
    for (const auto& wp : m_controlPoints) {
        points.push_back(wp.point);
    }
    return points;
}

std::vector<double> NurbsCurve::weights() const {
    std::vector<double> w;
    w.reserve(m_controlPoints.size());
    for (const auto& wp : m_controlPoints) {
        w.push_back(wp.weight);
    }
    return w;
}

void NurbsCurve::setControlPoint(int index, const Point& p) {
    if (index < 0 || index >= static_cast<int>(m_controlPoints.size())) {
        throw std::out_of_range("Control point index out of range");
    }
    m_controlPoints[index].point = p;
    updateHomogeneousCurve();
}

void NurbsCurve::setWeight(int index, double w) {
    if (index < 0 || index >= static_cast<int>(m_controlPoints.size())) {
        throw std::out_of_range("Control point index out of range");
    }
    m_controlPoints[index].weight = w;
    updateHomogeneousCurve();
}

void NurbsCurve::setControlPointAndWeight(int index, const Point& p, double w) {
    if (index < 0 || index >= static_cast<int>(m_controlPoints.size())) {
        throw std::out_of_range("Control point index out of range");
    }
    m_controlPoints[index] = WeightedPoint(p, w);
    updateHomogeneousCurve();
}

// ============== Factory 函数 ==============

namespace NurbsFactory 
{

/**
 * 创建圆弧
 * 
 * 使用二次 NURBS 表示圆弧
 * 根据弧度大小选择分段数
 */
NurbsCurve createArc(const Point& center, double radius,
                     double startAngle, double endAngle,
                     const Point& normal) {
    // 确保角度范围合理
    while (endAngle <= startAngle) {
        endAngle += 2.0 * M_PI;
    }
    
    double theta = endAngle - startAngle;
    
    // 根据弧度决定分段数（每段最多 90 度）
    int numArcs = static_cast<int>(std::ceil(theta / (M_PI / 2.0)));
    if (numArcs < 1) numArcs = 1;
    if (numArcs > 4) numArcs = 4;
    
    double dTheta = theta / numArcs;
    double w1 = std::cos(dTheta / 2.0);  // 中间控制点的权重
    
    // 构建控制点
    WeightedPointList controlPoints;
    
    // 计算局部坐标系
    Point zAxis = normal.normalized();
    Point xAxis, yAxis;
    
    // 构建正交基
    if (std::abs(zAxis.x) < 0.9) {
        xAxis = Point(1, 0, 0).cross(zAxis).normalized();
    } else 
{
        xAxis = Point(0, 1, 0).cross(zAxis).normalized();
    }
    yAxis = zAxis.cross(xAxis);
    
    double angle = startAngle;
    Point p0 = center + xAxis * (radius * std::cos(angle)) + 
                        yAxis * (radius * std::sin(angle));
    controlPoints.emplace_back(p0, 1.0);
    
    for (int i = 0; i < numArcs; i++) {
        double angleMid = angle + dTheta / 2.0;
        double angleEnd = angle + dTheta;
        
        // 中间控制点（在切线交点处）
        Point t0(-std::sin(angle), std::cos(angle), 0);
        Point t1(-std::sin(angleEnd), std::cos(angleEnd), 0);
        
        // 切线交点（简化计算）
        double r = radius / w1;
        Point pMid = center + xAxis * (r * std::cos(angleMid)) +
                              yAxis * (r * std::sin(angleMid));
        controlPoints.emplace_back(pMid, w1);
        
        // 端点
        Point pEnd = center + xAxis * (radius * std::cos(angleEnd)) +
                              yAxis * (radius * std::sin(angleEnd));
        controlPoints.emplace_back(pEnd, 1.0);
        
        angle = angleEnd;
    }
    
    // 构建节点向量
    // 对于 n+1 个控制点的二次曲线：m = n + 3
    int n = static_cast<int>(controlPoints.size()) - 1;
    std::vector<double> knots;
    
    // Clamped 节点向量
    knots.push_back(0.0);
    knots.push_back(0.0);
    knots.push_back(0.0);
    
    for (int i = 1; i < numArcs; i++) {
        double knotVal = static_cast<double>(i) / numArcs;
        knots.push_back(knotVal);
        knots.push_back(knotVal);
    }
    
    knots.push_back(1.0);
    knots.push_back(1.0);
    knots.push_back(1.0);
    
    return NurbsCurve(controlPoints, 2, knots);
}

NurbsCurve createCircle(const Point& center, double radius, const Point& normal) {
    return createArc(center, radius, 0.0, 2.0 * M_PI, normal);
}

NurbsCurve createEllipse(const Point& center, double semiMajor, double semiMinor,
                         const Point& normal) {
    // 椭圆可以通过缩放圆来创建
    // 或者直接构建 NURBS 表示
    
    // 使用 4 段二次 NURBS
    double w = std::sqrt(2.0) / 2.0;  // = cos(45°)
    
    WeightedPointList controlPoints = {
        {Point(center.x + semiMajor, center.y, center.z), 1.0},
        {Point(center.x + semiMajor, center.y + semiMinor, center.z), w},
        {Point(center.x, center.y + semiMinor, center.z), 1.0},
        {Point(center.x - semiMajor, center.y + semiMinor, center.z), w},
        {Point(center.x - semiMajor, center.y, center.z), 1.0},
        {Point(center.x - semiMajor, center.y - semiMinor, center.z), w},
        {Point(center.x, center.y - semiMinor, center.z), 1.0},
        {Point(center.x + semiMajor, center.y - semiMinor, center.z), w},
        {Point(center.x + semiMajor, center.y, center.z), 1.0}  // 回到起点
    };
    
    std::vector<double> knots = {
        0, 0, 0,
        0.25, 0.25,
        0.5, 0.5,
        0.75, 0.75,
        1, 1, 1
    };
    
    return NurbsCurve(controlPoints, 2, knots);
}

} // namespace NurbsFactory

} // namespace nurbs
