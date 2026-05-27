#include "core/curves/bezier_curve.hpp"
#include <cmath>
#include <algorithm>

namespace nurbs 
{

//==============================================================================
// BezierCurve 实现 (Section 1.3)
//==============================================================================

BezierCurve::BezierCurve(const PointList& controlPoints) 
    : m_controlPoints(controlPoints) 
{
    if (controlPoints.size() < 2)
        throw std::invalid_argument("Bézier curve requires at least 2 control points");
}

/**
 * Algorithm A1.1: de Casteljau 算法
 * 
 * 这是 Bézier 曲线求值的核心算法，具有以下优点:
 * 1. 数值稳定性好 (只使用线性插值)
 * 2. 几何意义直观
 * 3. 副产品丰富 (细分、导数)
 * 
 * 时间复杂度: O(n²)
 * 空间复杂度: O(n) (原地算法)
 */
Point BezierCurve::evaluate(double u) const 
{
    const int n = degree();
    // 复制控制点到工作数组
    PointList Q = m_controlPoints;
    // n 轮迭代，每轮减少一个点
    for (int r = 1; r <= n; r++) 
    {
        for (int i = 0; i <= n - r; i++) 
        { // 核心操作: 线性插值 (1-u)·Q[i] + u·Q[i+1]
            Q[i] = Point::lerp(Q[i], Q[i + 1], u);
        }
    }
    return Q[0];
}

/**
 * 返回完整的 de Casteljau 金字塔
 * 
 * 结构示意 (n=3, 三次曲线):
 * 
 *   Level 0:  P0 ----- P1 ----- P2 ----- P3    (原始控制点)
 *                 ↘   ↙    ↘   ↙    ↘   ↙
 *   Level 1:     P0[1] --- P1[1] --- P2[1]
 *                     ↘   ↙    ↘   ↙
 *   Level 2:         P0[2] --- P1[2]
 *                         ↘   ↙
 *   Level 3:             P0[3] = C(u)
 */
std::vector<PointList> BezierCurve::deCasteljauPyramid(double u) const 
{
    const int n = degree();
    std::vector<PointList> pyramid;
    
    // Level 0: 原始控制点
    pyramid.push_back(m_controlPoints);
    
    for (int r = 1; r <= n; r++) 
    { // 逐层计算
        PointList level;
        const PointList& prev = pyramid[r - 1];
        for (size_t i = 0; i < prev.size() - 1; i++) 
        { // 线性插值
            level.push_back(Point::lerp(prev[i], prev[i + 1], u));
        }
        pyramid.push_back(level);
    }
    return pyramid;
}

/**
 * 使用 Bernstein 基函数直接求值
 * 
 * C(u) = Σ B_{i,n}(u) · P_i
 * 
 * 这种方法直接套公式，但:
 * 1. 涉及幂运算和组合数，数值稳定性差
 * 2. 当 n 很大时，容易溢出
 * 3. 主要用于理论验证和对比
 */
Point BezierCurve::evaluateBernstein(double u) const 
{
    const int n = degree();
    Point result(0, 0, 0);
    for (int i = 0; i <= n; i++) 
    {
        double B = bernsteinBasis(i, n, u);
        result += m_controlPoints[i] * B;
    }
    return result;
}

double BezierCurve::binomial(int n, int k) 
{
    if (k < 0 || k > n) return 0;
    if (k == 0 || k == n) return 1;
    
    // 使用递推避免大数: C(n,k) = C(n,k-1) * (n-k+1) / k
    double result = 1.0;
    for (int i = 0; i < k; i++) 
    {
        result *= static_cast<double>(n - i) / (i + 1);
    }
    return result;
}

double BezierCurve::bernsteinBasis(int i, int n, double u) 
{
    return binomial(n, i) * std::pow(u, i) * std::pow(1.0 - u, n - i);
}

/**
 * 曲线细分 (Section 1.6)
 * 
 * de Casteljau 算法的一个重要副产品:
 * 在参数 u 处，金字塔的左边界和右边界分别是两段子曲线的控制点
 * 
 * 左曲线: 从 u=0 到 u=u_split
 * 右曲线: 从 u=u_split 到 u=1
 */
std::pair<PointList, PointList> BezierCurve::subdivide(double u) const 
{
    auto pyramid = deCasteljauPyramid(u);
    const int n = degree();
    
    PointList left, right;
    
    // 左曲线: 金字塔每层的第一个点
    // P_0^{[0]}, P_0^{[1]}, ..., P_0^{[n]}
    for (int r = 0; r <= n; r++) 
    {
        left.push_back(pyramid[r][0]);
    }
    
    // 右曲线: 金字塔每层的最后一个点 (从下往上)
    // P_0^{[n]}, P_1^{[n-1]}, ..., P_n^{[0]}
    for (int r = n; r >= 0; r--) 
    {
        right.push_back(pyramid[r].back());
    }
    
    return {left, right};
}

/**
 * 导数 (Section 1.3, Equation 1.12)
 * 
 * Bézier 曲线的一个优美性质:
 * n 次 Bézier 曲线的 k 阶导数是 (n-k) 次 Bézier 曲线
 * 
 * 一阶导数控制点: Q_i = n · (P_{i+1} - P_i)
 * 
 * 对于 de Casteljau: C'(u) 可以从金字塔直接计算
 */
Point BezierCurve::derivative(double u, int order) const 
{
    if (order <= 0)
        return evaluate(u);
    
    PointList derivPts = derivativeControlPoints(order);
    if (derivPts.empty()) 
        return Point(0, 0, 0);  // 导数为常数0
    
    // 用新的控制点求值
    BezierCurve derivCurve(derivPts);
    return derivCurve.evaluate(u);
}

PointList BezierCurve::derivativeControlPoints(int order) const 
{
    if (order <= 0)
        return m_controlPoints;
    
    const int n = degree();
    if (order > n)
        return {};  // 超过次数，导数为0
    
    // 递推: 每求一阶导数，次数降1
    PointList pts = m_controlPoints;
    
    for (int k = 0; k < order; k++) 
    {
        int m = static_cast<int>(pts.size()) - 1;  // 当前次数
        PointList newPts;
        
        for (int i = 0; i < m; i++) 
        { // Q_i = m · (P_{i+1} - P_i)
            newPts.push_back((pts[i + 1] - pts[i]) * m);
        }
        pts = newPts;
    }
    
    return pts;
}

/**
 * 升阶 (Section 1.7, Equation 1.17)
 * 
 * 将 n 次曲线表示为 n+1 次曲线，形状完全不变
 * 
 * 新控制点:
 *   P_0^{new} = P_0
 *   P_i^{new} = (i/(n+1)) · P_{i-1} + (1 - i/(n+1)) · P_i,  i=1,...,n
 *   P_{n+1}^{new} = P_n
 */
BezierCurve BezierCurve::degreeElevate() const 
{
    const int n = degree();
    PointList newPoints(n + 2);
    
    // 端点不变
    newPoints[0] = m_controlPoints[0];
    newPoints[n + 1] = m_controlPoints[n];
    
    // 中间点
    for (int i = 1; i <= n; i++) 
    {
        double alpha = static_cast<double>(i) / (n + 1);
        newPoints[i] = m_controlPoints[i - 1] * alpha + 
                       m_controlPoints[i] * (1.0 - alpha);
    }
    
    return BezierCurve(newPoints);
}

PointList BezierCurve::sample(int numSamples) const 
{
    PointList points;
    points.reserve(numSamples);
    // Calculate points on the curve at regular intervals of u
    for (int i = 0; i < numSamples; i++) 
    {
        double u = static_cast<double>(i) / (numSamples - 1);
        points.push_back(evaluate(u));
    } 
    return points;
}

void BezierCurve::setControlPoint(int index, const Point& p) 
{
    if (index < 0 || index >= static_cast<int>(m_controlPoints.size())) 
        throw std::out_of_range("Control point index out of range");

    m_controlPoints[index] = p;
}

void BezierCurve::setControlPoints(const PointList& points) {
    if (points.size() < 2)
        throw std::invalid_argument("Bézier curve requires at least 2 control points");

    m_controlPoints = points;
}


//==============================================================================
// RationalBezierCurve 实现 (Section 1.4)
//==============================================================================

RationalBezierCurve::RationalBezierCurve(const PointList& controlPoints, const std::vector<double>& weights)
    : m_controlPoints(controlPoints), m_weights(weights)
{
    if (controlPoints.size() < 2)
        throw std::invalid_argument("Rational Bézier curve requires at least 2 control points");
    
    if (controlPoints.size() != weights.size())
        throw std::invalid_argument("Number of weights must match number of control points");
}

RationalBezierCurve::RationalBezierCurve(const PointList& controlPoints)
    : m_controlPoints(controlPoints), m_weights(controlPoints.size(), 1.0)  // 所有权重为1
{
    if (controlPoints.size() < 2) 
        throw std::invalid_argument("Rational Bézier curve requires at least 2 control points");
}

std::vector<RationalBezierCurve::HomogeneousPoint> RationalBezierCurve::toHomogeneous() const 
{
    std::vector<HomogeneousPoint> result;
    result.reserve(m_controlPoints.size());
    
    for (size_t i = 0; i < m_controlPoints.size(); i++) 
    {
        result.emplace_back(m_controlPoints[i], m_weights[i]);
    } 
    return result;
}

/**
 * Algorithm A1.2: 有理 de Casteljau 算法 (Page 28)
 * 
 * 核心思想: 在齐次坐标空间做 de Casteljau，然后投影
 * 
 * 这利用了一个关键性质:
 *   有理曲线 = 高维非有理曲线的投影
 * 
 * 具体步骤:
 * 1. (P_i, w_i) → (w_i·x_i, w_i·y_i, w_i·z_i, w_i)  升维到齐次空间
 * 2. 在 4D 空间做标准 de Casteljau (线性插值)
 * 3. (X, Y, Z, W) → (X/W, Y/W, Z/W)  透视除法回到 3D
 */
Point RationalBezierCurve::evaluate(double u) const 
{
    const int n = degree();
    
    // 转换为齐次坐标
    auto Q = toHomogeneous();
    
    // 在齐次空间做 de Casteljau
    for (int r = 1; r <= n; r++) 
    {
        for (int i = 0; i <= n - r; i++) 
        {
            Q[i] = HomogeneousPoint::lerp(Q[i], Q[i + 1], u);
        }
    }
    
    // 透视除法
    return Q[0].project();
}

/**
 * 有理基函数 R_{i,n}(u)
 * 
 * Equation 1.19:
 *   R_{i,n}(u) = (B_{i,n}(u) · w_i) / (Σ_j B_{j,n}(u) · w_j)
 */
double RationalBezierCurve::rationalBasis(int i, double u) const 
{
    const int n = degree();
    
    // 分子: B_{i,n}(u) · w_i
    double numerator = BezierCurve::bernsteinBasis(i, n, u) * m_weights[i];
    
    // 分母: Σ B_{j,n}(u) · w_j
    double denominator = 0.0;
    for (int j = 0; j <= n; j++) 
    {
        denominator += BezierCurve::bernsteinBasis(j, n, u) * m_weights[j];
    }
    
    if (std::abs(denominator) < 1e-10)
        return 0.0;
    return numerator / denominator;
}

/**
 * 返回有理 de Casteljau 金字塔 (投影后的 3D 点)
 * 
 * 用于可视化有理曲线的构造过程
 */
std::vector<PointList> RationalBezierCurve::deCasteljauPyramid(double u) const 
{
    const int n = degree();
    std::vector<PointList> pyramid;
    
    // 在齐次空间计算
    std::vector<std::vector<HomogeneousPoint>> homoPyramid;
    homoPyramid.push_back(toHomogeneous());
    
    for (int r = 1; r <= n; r++) 
    {
        std::vector<HomogeneousPoint> level;
        const auto& prev = homoPyramid[r - 1];
        
        for (size_t i = 0; i < prev.size() - 1; i++)
            level.push_back(HomogeneousPoint::lerp(prev[i], prev[i + 1], u));
        
        homoPyramid.push_back(level);
    }
    
    // 投影到 3D
    for (const auto& homoLevel : homoPyramid) 
    {
        PointList level;
        for (const auto& hp : homoLevel) 
        {
            level.push_back(hp.project());
        }
        pyramid.push_back(level);
    }
    
    return pyramid;
}

/**
 * 有理曲线导数 (Section 1.4.2)
 * 
 * 由于是商的形式，导数比较复杂:
 * 
 * 设 A(u) = Σ B_{i,n}(u) · w_i · P_i  (分子)
 *    w(u) = Σ B_{i,n}(u) · w_i        (分母)
 *    C(u) = A(u) / w(u)
 * 
 * 则 C'(u) = (A'(u) - C(u)·w'(u)) / w(u)
 */
Point RationalBezierCurve::derivative(double u, int order) const 
{
    if (order <= 0)
        return evaluate(u);
    
    const int n = degree();
    
    // 计算 w(u) 和 A(u)
    double w_u = 0.0;
    Point A_u(0, 0, 0);
    
    for (int i = 0; i <= n; i++) 
    {
        double B = BezierCurve::bernsteinBasis(i, n, u);
        w_u += B * m_weights[i];
        A_u += m_controlPoints[i] * (B * m_weights[i]);
    }
    
    if (std::abs(w_u) < 1e-10)
        return Point(0, 0, 0);
    
    Point C_u = A_u / w_u;
    if (order == 1) 
    {   // 计算 A'(u) 和 w'(u)
        // A(u) 和 w(u) 都是 n 次 Bézier 曲线，其导数是 n-1 次
        Point A_prime(0, 0, 0);
        double w_prime = 0.0;
        
        for (int i = 0; i < n; i++) 
        { // 导数控制点: n * (P_{i+1}·w_{i+1} - P_i·w_i)
            Point dA = (m_controlPoints[i + 1] * m_weights[i + 1] - 
                        m_controlPoints[i] * m_weights[i]) * n;
            double dw = (m_weights[i + 1] - m_weights[i]) * n;
            double B = BezierCurve::bernsteinBasis(i, n - 1, u);
            A_prime += dA * B;
            w_prime += dw * B;
        }
        
        // C'(u) = (A'(u) - C(u)·w'(u)) / w(u)
        return (A_prime - C_u * w_prime) / w_u;
    }
    
    // 高阶导数可以递归计算，这里简化处理
    // TODO: 实现完整的高阶导数
    return Point(0, 0, 0);
}

/**
 * 有理曲线细分
 * 
 * 在齐次空间细分，然后转换回来
 * 关键点: 细分后的控制点权重需要从齐次坐标提取
 */
std::pair<RationalBezierCurve, RationalBezierCurve> 
RationalBezierCurve::subdivide(double u) const 
{
    const int n = degree();
    
    // 在齐次空间计算金字塔
    std::vector<std::vector<HomogeneousPoint>> pyramid;
    pyramid.push_back(toHomogeneous());
    
    for (int r = 1; r <= n; r++) 
    {
        std::vector<HomogeneousPoint> level;
        const auto& prev = pyramid[r - 1];
        
        for (size_t i = 0; i < prev.size() - 1; i++) 
        {
            level.push_back(HomogeneousPoint::lerp(prev[i], prev[i + 1], u));
        }
        pyramid.push_back(level);
    }
    
    // 提取左曲线
    PointList leftPts;
    std::vector<double> leftWeights;
    for (int r = 0; r <= n; r++) 
    {
        const auto& hp = pyramid[r][0];
        leftPts.push_back(hp.project());
        leftWeights.push_back(hp.weight());
    }
    
    // 提取右曲线
    PointList rightPts;
    std::vector<double> rightWeights;
    for (int r = n; r >= 0; r--) 
    {
        const auto& hp = pyramid[r].back();
        rightPts.push_back(hp.project());
        rightWeights.push_back(hp.weight());
    }
    
    return {
        RationalBezierCurve(leftPts, leftWeights),
        RationalBezierCurve(rightPts, rightWeights)
    };
}

PointList RationalBezierCurve::sample(int numSamples) const 
{
    PointList points;
    points.reserve(numSamples);
    for (int i = 0; i < numSamples; i++) 
    {
        double u = static_cast<double>(i) / (numSamples - 1);
        points.push_back(evaluate(u));
    }
    return points;
}

void RationalBezierCurve::setControlPoint(int index, const Point& p) 
{
    if (index < 0 || index >= static_cast<int>(m_controlPoints.size())) 
        throw std::out_of_range("Control point index out of range");
    
        m_controlPoints[index] = p;
}

void RationalBezierCurve::setWeight(int index, double w) 
{
    if (index < 0 || index >= static_cast<int>(m_weights.size()))
        throw std::out_of_range("Weight index out of range");
    
    m_weights[index] = w;
}


//==============================================================================
// ConicFactory 实现 (Section 1.4.3)
//==============================================================================

namespace ConicFactory 
{

/**
 * 创建圆弧 (二次有理 Bézier)
 * 
 * The NURBS Book Section 1.4.3, Example 1.5
 * 
 * 对于弧度角 θ 的圆弧:
 *   P0 = 起点
 *   P1 = 两端切线的交点
 *   P2 = 终点
 *   w0 = w2 = 1
 *   w1 = cos(θ/2)
 * 
 * 关键洞察:
 *   当 w1 = cos(θ/2) 时，二次有理 Bézier 精确表示圆弧
 *   当 θ > 180° 时，w1 < 0，通常需要分段
 */
RationalBezierCurve createArc(const Point& center, double radius,
                               double startAngle, double endAngle) 
{
    // 计算弧度角
    double theta = endAngle - startAngle;
    
    // 如果弧度太大，需要分段 (这里简化处理，假设 θ ≤ 180°)
    if (std::abs(theta) > M_PI) 
    { // 实际应用中应该分段，这里截断
        theta = (theta > 0) ? M_PI : -M_PI;
        endAngle = startAngle + theta;
    }
    
    // 起点和终点
    Point P0 = center + Point(radius * std::cos(startAngle), 
                               radius * std::sin(startAngle), 0);
    Point P2 = center + Point(radius * std::cos(endAngle), 
                               radius * std::sin(endAngle), 0);
    
    // 中间控制点: 两端切线的交点
    // 切线方向: 圆上点的切向量是 (-sin, cos)
    // 交点计算: 解两条切线的交点
    double halfTheta = theta / 2.0;
    double midAngle = (startAngle + endAngle) / 2.0;
    
    // 交点到圆心的距离为 r / cos(θ/2)
    double dist = radius / std::cos(halfTheta);
    Point P1 = center + Point(dist * std::cos(midAngle), 
                               dist * std::sin(midAngle), 0);
    
    // 中间点权重
    double w1 = std::cos(halfTheta);
    
    return RationalBezierCurve({P0, P1, P2}, {1.0, w1, 1.0});
}

/**
 * 创建 90° 圆弧
 * 
 * quadrant: 0=第一象限, 1=第二象限, 2=第三象限, 3=第四象限
 * 
 * 对于 90° 圆弧: w1 = cos(45°) = √2/2 ≈ 0.7071
 */
RationalBezierCurve createQuarterCircle(const Point& center, double radius,
                                         int quadrant) 
{
    double startAngle = quadrant * M_PI / 2.0;
    double endAngle = startAngle + M_PI / 2.0;
    
    return createArc(center, radius, startAngle, endAngle);
}

/**
 * 创建完整圆 (4 段 90° 圆弧)
 * 
 * 圆不能用单一有理 Bézier 曲线精确表示
 * 需要至少 4 段（每段 ≤ 90°）
 */
std::vector<RationalBezierCurve> createFullCircle(const Point& center, double radius) 
{
    std::vector<RationalBezierCurve> arcs;
    for (int i = 0; i < 4; i++) 
    {
        arcs.push_back(createQuarterCircle(center, radius, i));
    }
    return arcs;
}

/**
 * 创建椭圆弧
 * 
 * 椭圆: (x/a)² + (y/b)² = 1
 * 
 * 方法: 先创建单位圆弧，然后缩放
 */
RationalBezierCurve createEllipticArc(const Point& center,
                                       double semiMajor, double semiMinor,
                                       double startAngle, double endAngle) 
{
    // 先创建单位圆弧
    auto unitArc = createArc(Point(0, 0, 0), 1.0, startAngle, endAngle);
    
    // 缩放控制点
    PointList scaledPts;
    for (const auto& p : unitArc.controlPoints()) 
    {
        scaledPts.push_back(Point(center.x + p.x * semiMajor,
                                  center.y + p.y * semiMinor,
                                  center.z + p.z));
    }
    // 权重不变
    return RationalBezierCurve(scaledPts, unitArc.weights());
}

} // namespace ConicFactory

} // namespace nurbs
