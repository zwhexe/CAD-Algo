#include "core/curves/bspline_curve.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace nurbs 
{

BSplineCurve::BSplineCurve(const PointList& controlPoints, int degree, const std::vector<double>& knots)
    : m_controlPoints(controlPoints)
    , m_knots(knots)
    , m_degree(degree)
{
    if (controlPoints.size() < 2) {
        throw std::invalid_argument("B-spline requires at least 2 control points");
    }
    if (degree < 1) {
        throw std::invalid_argument("B-spline degree must be at least 1");
    }
    validateKnotVector();
}

BSplineCurve::BSplineCurve(const PointList& controlPoints, int degree)
    : m_controlPoints(controlPoints)
    , m_degree(degree)
{
    if (controlPoints.size() < 2) {
        throw std::invalid_argument("B-spline requires at least 2 control points");
    }
    if (degree < 1) {
        throw std::invalid_argument("B-spline degree must be at least 1");
    }
    
    int n = static_cast<int>(controlPoints.size()) - 1;
    if (degree > n) {
        throw std::invalid_argument("Degree cannot exceed n (number of control points - 1)");
    }
    
    m_knots = generateClampedUniformKnots(n, degree);
}

void BSplineCurve::validateKnotVector() const {
    int n = numControlPoints() - 1;
    int m = numKnots() - 1;
    
    // 基本关系: m = n + p + 1
    if (m != n + m_degree + 1) {
        throw std::invalid_argument(
            "Invalid knot vector size. Expected " + std::to_string(n + m_degree + 2) +
            " knots, got " + std::to_string(m + 1));
    }
    
    // 节点向量必须非递减
    for (size_t i = 1; i < m_knots.size(); i++) {
        if (m_knots[i] < m_knots[i-1]) {
            throw std::invalid_argument("Knot vector must be non-decreasing");
        }
    }
}

std::vector<double> BSplineCurve::generateClampedUniformKnots(int n, int p) {
    // Clamped uniform knot vector:
    // [0, 0, ..., 0, 1/(n-p+1), 2/(n-p+1), ..., 1, 1, ..., 1]
    //  \_p+1个_/                              \_p+1个_/
    
    int m = n + p + 1;
    std::vector<double> knots(m + 1);
    
    // 前 p+1 个节点为 0
    for (int i = 0; i <= p; i++) {
        knots[i] = 0.0;
    }
    
    // 中间节点均匀分布
    int numInternalKnots = m - 2 * p - 1;
    for (int i = 1; i <= numInternalKnots; i++) {
        knots[p + i] = static_cast<double>(i) / (numInternalKnots + 1);
    }
    
    // 后 p+1 个节点为 1
    for (int i = m - p; i <= m; i++) {
        knots[i] = 1.0;
    }
    
    return knots;
}

/**
 * Algorithm A2.1 - 查找参数 u 所在的节点区间
 * 
 * 返回索引 i，使得 u ∈ [u_i, u_{i+1})
 * 对于 u = u_max，返回 n（最后一个有效区间）
 */
int BSplineCurve::findSpan(double u) const {
    int n = numControlPoints() - 1;
    
    // 特殊情况：u 在上边界
    if (u >= m_knots[n + 1]) {
        return n;
    }
    
    // 特殊情况：u 在下边界
    if (u <= m_knots[m_degree]) {
        return m_degree;
    }
    
    // 二分查找
    int low = m_degree;
    int high = n + 1;
    int mid = (low + high) / 2;
    
    while (u < m_knots[mid] || u >= m_knots[mid + 1]) {
        if (u < m_knots[mid]) {
            high = mid;
        } else 
{
            low = mid;
        }
        mid = (low + high) / 2;
    }
    
    return mid;
}

/**
 * Algorithm A2.2 - 计算单个基函数 N_{i,p}(u)
 * 
 * 使用递推公式：
 *   N_{i,0}(u) = 1 if u_i <= u < u_{i+1}, else 0
 *   N_{i,p}(u) = (u - u_i)/(u_{i+p} - u_i) * N_{i,p-1}(u)
 *             + (u_{i+p+1} - u)/(u_{i+p+1} - u_{i+1}) * N_{i+1,p-1}(u)
 */
double BSplineCurve::basisFunction(int i, int p, double u) const {
    // 0 次基函数
    if (p == 0) {
        if (u >= m_knots[i] && u < m_knots[i + 1]) {
            return 1.0;
        }
        // 特殊处理：u 在最后一个节点
        if (i == numControlPoints() - 1 && u == m_knots[i + 1]) {
            return 1.0;
        }
        return 0.0;
    }
    
    // 递推
    double left = 0.0, right = 0.0;
    
    double denom1 = m_knots[i + p] - m_knots[i];
    if (denom1 > 1e-10) {
        left = (u - m_knots[i]) / denom1 * basisFunction(i, p - 1, u);
    }
    
    double denom2 = m_knots[i + p + 1] - m_knots[i + 1];
    if (denom2 > 1e-10) {
        right = (m_knots[i + p + 1] - u) / denom2 * basisFunction(i + 1, p - 1, u);
    }
    
    return left + right;
}

/**
 * Algorithm A2.4 - 计算所有非零基函数（更高效）
 * 
 * 对于给定的 u，最多只有 p+1 个基函数非零
 * 返回 [N_{span-p,p}(u), ..., N_{span,p}(u)]
 */
std::vector<double> BSplineCurve::basisFunctions(int span, double u) const {
    std::vector<double> N(m_degree + 1);
    std::vector<double> left(m_degree + 1);
    std::vector<double> right(m_degree + 1);
    
    N[0] = 1.0;
    
    for (int j = 1; j <= m_degree; j++) {
        left[j] = u - m_knots[span + 1 - j];
        right[j] = m_knots[span + j] - u;
        
        double saved = 0.0;
        for (int r = 0; r < j; r++) {
            double temp = N[r] / (right[r + 1] + left[j - r]);
            N[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        N[j] = saved;
    }
    
    return N;
}

/**
 * Algorithm A3.1 - de Boor 算法求值
 * 
 * 这是 B-spline 版本的 de Casteljau 算法
 * 
 * 步骤：
 * 1. 找到 u 所在的节点区间 [u_k, u_{k+1})
 * 2. 提取受影响的 p+1 个控制点
 * 3. 进行 p 轮线性插值
 */
Point BSplineCurve::evaluate(double u) const {
    // 限制 u 在有效范围内
    u = std::max(uMin(), std::min(uMax(), u));
    
    int span = findSpan(u);
    std::vector<double> N = basisFunctions(span, u);
    
    Point result(0, 0, 0);
    for (int i = 0; i <= m_degree; i++) {
        result += m_controlPoints[span - m_degree + i] * N[i];
    }
    
    return result;
}

/**
 * de Boor 算法的完整金字塔
 * 用于可视化理解 de Boor 算法的几何意义
 */
std::vector<PointList> BSplineCurve::deBoorPyramid(double u) const {
    u = std::max(uMin(), std::min(uMax(), u));
    
    int span = findSpan(u);
    std::vector<PointList> pyramid;
    
    // Level 0: 受影响的原始控制点
    PointList level0;
    for (int i = span - m_degree; i <= span; i++) {
        level0.push_back(m_controlPoints[i]);
    }
    pyramid.push_back(level0);
    
    // 复制用于迭代
    PointList d = level0;
    
    // p 轮插值
    for (int r = 1; r <= m_degree; r++) {
        PointList level;
        for (int i = 0; i <= m_degree - r; i++) {
            int globalIdx = span - m_degree + r + i;
            double alpha = (u - m_knots[globalIdx]) / 
                          (m_knots[globalIdx + m_degree - r + 1] - m_knots[globalIdx]);
            d[i] = d[i] * (1.0 - alpha) + d[i + 1] * alpha;
            level.push_back(d[i]);
        }
        pyramid.push_back(level);
    }
    
    return pyramid;
}

/**
 * Algorithm A5.1 - 节点插入
 * 
 * 在不改变曲线形状的情况下插入新节点
 * 这是很多 B-spline 算法的基础（如细分、degree elevation 等）
 */
BSplineCurve BSplineCurve::knotInsert(double u) const {
    int k = findSpan(u);
    int n = numControlPoints() - 1;
    
    // 新的控制点 (n+2 个)
    PointList newPoints(n + 2);
    
    // 新的节点向量 (多一个节点)
    std::vector<double> newKnots;
    newKnots.reserve(m_knots.size() + 1);
    
    // 插入节点
    for (size_t i = 0; i <= static_cast<size_t>(k); i++) {
        newKnots.push_back(m_knots[i]);
    }
    newKnots.push_back(u);
    for (size_t i = k + 1; i < m_knots.size(); i++) {
        newKnots.push_back(m_knots[i]);
    }
    
    // 计算新控制点
    // 前 k-p+1 个点不变
    for (int i = 0; i <= k - m_degree; i++) {
        newPoints[i] = m_controlPoints[i];
    }
    
    // 后 n-k 个点不变（索引右移1）
    for (int i = k; i <= n; i++) {
        newPoints[i + 1] = m_controlPoints[i];
    }
    
    // 中间 p 个点需要重新计算
    for (int i = k - m_degree + 1; i <= k; i++) {
        double alpha = (u - m_knots[i]) / (m_knots[i + m_degree] - m_knots[i]);
        newPoints[i] = m_controlPoints[i - 1] * (1.0 - alpha) + m_controlPoints[i] * alpha;
    }
    
    return BSplineCurve(newPoints, m_degree, newKnots);
}

PointList BSplineCurve::sample(int numSamples) const {
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

void BSplineCurve::setControlPoint(int index, const Point& p) {
    if (index < 0 || index >= static_cast<int>(m_controlPoints.size())) {
        throw std::out_of_range("Control point index out of range");
    }
    m_controlPoints[index] = p;
}

// Factory 函数
namespace BSplineFactory 
{

BSplineCurve createUniform(const PointList& controlPoints, int degree) {
    return BSplineCurve(controlPoints, degree);
}

} // namespace BSplineFactory

} // namespace nurbs
