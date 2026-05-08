#include "core/surfaces/bezier_surface.hpp"
#include <cmath>
#include <algorithm>
#include <array>

namespace nurbs 
{

//==============================================================================
// BezierSurface 实现 (Section 1.5)
//==============================================================================

BezierSurface::BezierSurface(const ControlPointGrid& controlPoints)
    : m_controlPoints(controlPoints)
{
    validateGrid();
}

BezierSurface::BezierSurface(int degreeU, int degreeV) {
    // 创建空网格
    m_controlPoints.resize(degreeU + 1);
    for (auto& row : m_controlPoints) {
        row.resize(degreeV + 1);
    }
}

void BezierSurface::validateGrid() const {
    if (m_controlPoints.empty()) 
{
        throw std::invalid_argument("Control point grid cannot be empty");
    }
    
    size_t cols = m_controlPoints[0].size();
    if (cols == 0) {
        throw std::invalid_argument("Control point grid must have at least 1x1 points");
    }
    
    for (const auto& row : m_controlPoints) {
        if (row.size() != cols) {
            throw std::invalid_argument("Control point grid must be rectangular");
        }
    }
}

/**
 * Algorithm A1.3: de Casteljau 曲面求值
 * 
 * 步骤:
 * 1. 对每一行 (共 n+1 行) 应用 de Casteljau(u)，得到 n+1 个点
 * 2. 对这 n+1 个点应用 de Casteljau(v)
 * 
 * 或者等价地:
 * 1. 对每一列应用 de Casteljau(v)
 * 2. 对结果应用 de Casteljau(u)
 * 
 * 时间复杂度: O(n·m)
 */
Point BezierSurface::evaluate(double u, double v) const {
    const int n = degreeU();
    const int m = degreeV();
    
    // 第一步: 对每一行做 de Casteljau(u)
    // 得到 m+1 个点 (每列一个)
    PointList tempPoints(m + 1);
    
    for (int j = 0; j <= m; j++) {
        // 提取第 j 列的控制点
        PointList colPoints(n + 1);
        for (int i = 0; i <= n; i++) {
            colPoints[i] = m_controlPoints[i][j];
        }
        
        // 对这一列做 de Casteljau(u)
        BezierCurve curve(colPoints);
        tempPoints[j] = curve.evaluate(u);
    }
    
    // 第二步: 对 tempPoints 做 de Casteljau(v)
    BezierCurve finalCurve(tempPoints);
    return finalCurve.evaluate(v);
}

/**
 * 完整求值: 位置 + 导数 + 法向量
 */
SurfacePoint BezierSurface::evaluateFull(double u, double v) const {
    SurfacePoint result;
    
    result.position = evaluate(u, v);
    result.derivativeU = derivativeU(u, v);
    result.derivativeV = derivativeV(u, v);
    result.normal = result.derivativeU.cross(result.derivativeV);
    
    return result;
}

/**
 * 固定 u 的等参线
 * 
 * 对每一行做 de Casteljau(u)，得到 m+1 个点
 * 这些点是 v 方向 Bézier 曲线的控制点
 */
BezierCurve BezierSurface::isoCurveU(double u) const {
    const int n = degreeU();
    const int m = degreeV();
    
    PointList isoPoints(m + 1);
    
    for (int j = 0; j <= m; j++) {
        // 提取第 j 列
        PointList colPoints(n + 1);
        for (int i = 0; i <= n; i++) {
            colPoints[i] = m_controlPoints[i][j];
        }
        
        BezierCurve curve(colPoints);
        isoPoints[j] = curve.evaluate(u);
    }
    
    return BezierCurve(isoPoints);
}

/**
 * 固定 v 的等参线
 */
BezierCurve BezierSurface::isoCurveV(double v) const {
    const int n = degreeU();
    const int m = degreeV();
    
    PointList isoPoints(n + 1);
    
    for (int i = 0; i <= n; i++) {
        // 第 i 行
        BezierCurve rowCurve(m_controlPoints[i]);
        isoPoints[i] = rowCurve.evaluate(v);
    }
    
    return BezierCurve(isoPoints);
}

/**
 * 边界曲线
 */
BezierCurve BezierSurface::boundaryBottom() const {
    // v = 0, 即第 0 列: P_{i,0}
    PointList pts;
    for (const auto& row : m_controlPoints) {
        pts.push_back(row[0]);
    }
    return BezierCurve(pts);
}

BezierCurve BezierSurface::boundaryTop() const {
    // v = 1, 即最后一列: P_{i,m}
    PointList pts;
    for (const auto& row : m_controlPoints) {
        pts.push_back(row.back());
    }
    return BezierCurve(pts);
}

BezierCurve BezierSurface::boundaryLeft() const {
    // u = 0, 即第 0 行: P_{0,j}
    return BezierCurve(m_controlPoints[0]);
}

BezierCurve BezierSurface::boundaryRight() const {
    // u = 1, 即最后一行: P_{n,j}
    return BezierCurve(m_controlPoints.back());
}

/**
 * 偏导数 ∂S/∂u
 * 
 * 使用差分控制点:
 *   Q_{i,j} = n · (P_{i+1,j} - P_{i,j})
 * 
 * 导数曲面是 (n-1) × m 次曲面
 */
Point BezierSurface::derivativeU(double u, double v) const {
    const int n = degreeU();
    const int m = degreeV();
    
    if (n == 0) {
        return Point(0, 0, 0);  // 常数曲面
    }
    
    // 构建导数曲面的控制点
    ControlPointGrid derivGrid(n);
    for (int i = 0; i < n; i++) {
        derivGrid[i].resize(m + 1);
        for (int j = 0; j <= m; j++) {
            derivGrid[i][j] = (m_controlPoints[i + 1][j] - m_controlPoints[i][j]) * n;
        }
    }
    
    BezierSurface derivSurface(derivGrid);
    return derivSurface.evaluate(u, v);
}

Point BezierSurface::derivativeV(double u, double v) const {
    const int n = degreeU();
    const int m = degreeV();
    
    if (m == 0) {
        return Point(0, 0, 0);
    }
    
    ControlPointGrid derivGrid(n + 1);
    for (int i = 0; i <= n; i++) {
        derivGrid[i].resize(m);
        for (int j = 0; j < m; j++) {
            derivGrid[i][j] = (m_controlPoints[i][j + 1] - m_controlPoints[i][j]) * m;
        }
    }
    
    BezierSurface derivSurface(derivGrid);
    return derivSurface.evaluate(u, v);
}

Point BezierSurface::normal(double u, double v) const {
    Point du = derivativeU(u, v);
    Point dv = derivativeV(u, v);
    return du.cross(dv);
}

/**
 * u 方向细分
 * 
 * 对每一行做曲线细分
 */
std::pair<BezierSurface, BezierSurface> BezierSurface::subdivideU(double u) const {
    const int n = degreeU();
    const int m = degreeV();
    
    ControlPointGrid leftGrid(n + 1), rightGrid(n + 1);
    for (int i = 0; i <= n; i++) {
        leftGrid[i].resize(m + 1);
        rightGrid[i].resize(m + 1);
    }
    
    // 对每列做细分
    for (int j = 0; j <= m; j++) {
        PointList colPoints(n + 1);
        for (int i = 0; i <= n; i++) {
            colPoints[i] = m_controlPoints[i][j];
        }
        
        BezierCurve curve(colPoints);
        auto [leftPts, rightPts] = curve.subdivide(u);
        
        for (int i = 0; i <= n; i++) {
            leftGrid[i][j] = leftPts[i];
            rightGrid[i][j] = rightPts[i];
        }
    }
    
    return {BezierSurface(leftGrid), BezierSurface(rightGrid)};
}

std::pair<BezierSurface, BezierSurface> BezierSurface::subdivideV(double v) const {
    const int n = degreeU();
    const int m = degreeV();
    
    ControlPointGrid bottomGrid(n + 1), topGrid(n + 1);
    for (int i = 0; i <= n; i++) {
        bottomGrid[i].resize(m + 1);
        topGrid[i].resize(m + 1);
    }
    
    // 对每行做细分
    for (int i = 0; i <= n; i++) {
        BezierCurve curve(m_controlPoints[i]);
        auto [bottomPts, topPts] = curve.subdivide(v);
        
        for (int j = 0; j <= m; j++) {
            bottomGrid[i][j] = bottomPts[j];
            topGrid[i][j] = topPts[j];
        }
    }
    
    return {BezierSurface(bottomGrid), BezierSurface(topGrid)};
}

std::vector<BezierSurface> BezierSurface::subdivide(double u, double v) const {
    auto [left, right] = subdivideU(u);
    auto [leftBottom, leftTop] = left.subdivideV(v);
    auto [rightBottom, rightTop] = right.subdivideV(v);
    
    return {leftBottom, rightBottom, leftTop, rightTop};
}

BezierSurface BezierSurface::degreeElevateU() const {
    const int n = degreeU();
    const int m = degreeV();
    
    ControlPointGrid newGrid(n + 2);
    for (auto& row : newGrid) {
        row.resize(m + 1);
    }
    
    // 对每列做升阶
    for (int j = 0; j <= m; j++) {
        PointList colPoints(n + 1);
        for (int i = 0; i <= n; i++) {
            colPoints[i] = m_controlPoints[i][j];
        }
        
        BezierCurve curve(colPoints);
        auto elevated = curve.degreeElevate();
        
        for (int i = 0; i <= n + 1; i++) {
            newGrid[i][j] = elevated.controlPoints()[i];
        }
    }
    
    return BezierSurface(newGrid);
}

BezierSurface BezierSurface::degreeElevateV() const {
    const int n = degreeU();
    const int m = degreeV();
    
    ControlPointGrid newGrid(n + 1);
    for (auto& row : newGrid) {
        row.resize(m + 2);
    }
    
    // 对每行做升阶
    for (int i = 0; i <= n; i++) {
        BezierCurve curve(m_controlPoints[i]);
        auto elevated = curve.degreeElevate();
        
        for (int j = 0; j <= m + 1; j++) {
            newGrid[i][j] = elevated.controlPoints()[j];
        }
    }
    
    return BezierSurface(newGrid);
}

ControlPointGrid BezierSurface::sampleGrid(int samplesU, int samplesV) const {
    ControlPointGrid grid(samplesU);
    
    for (int i = 0; i < samplesU; i++) {
        grid[i].resize(samplesV);
        double u = static_cast<double>(i) / (samplesU - 1);
        
        for (int j = 0; j < samplesV; j++) {
            double v = static_cast<double>(j) / (samplesV - 1);
            grid[i][j] = evaluate(u, v);
        }
    }
    
    return grid;
}

std::pair<std::vector<Point>, std::vector<std::array<int, 3>>>
BezierSurface::triangulate(int samplesU, int samplesV) const {
    auto grid = sampleGrid(samplesU, samplesV);
    
    // 展平为顶点数组
    std::vector<Point> vertices;
    vertices.reserve(samplesU * samplesV);
    
    for (int i = 0; i < samplesU; i++) {
        for (int j = 0; j < samplesV; j++) {
            vertices.push_back(grid[i][j]);
        }
    }
    
    // 生成三角形索引
    std::vector<std::array<int, 3>> triangles;
    triangles.reserve(2 * (samplesU - 1) * (samplesV - 1));
    
    auto idx = [samplesV](int i, int j) { return i * samplesV + j; };
    
    for (int i = 0; i < samplesU - 1; i++) {
        for (int j = 0; j < samplesV - 1; j++) {
            // 两个三角形组成一个四边形
            triangles.push_back({idx(i, j), idx(i+1, j), idx(i+1, j+1)});
            triangles.push_back({idx(i, j), idx(i+1, j+1), idx(i, j+1)});
        }
    }
    
    return {vertices, triangles};
}

void BezierSurface::setControlPoint(int i, int j, const Point& p) {
    if (i < 0 || i > degreeU() || j < 0 || j > degreeV()) {
        throw std::out_of_range("Control point index out of range");
    }
    m_controlPoints[i][j] = p;
}


//==============================================================================
// RationalBezierSurface 实现
//==============================================================================

RationalBezierSurface::RationalBezierSurface(const ControlPointGrid& controlPoints,
                                             const std::vector<std::vector<double>>& weights) {
    if (controlPoints.empty() || controlPoints[0].empty()) {
        throw std::invalid_argument("Control point grid cannot be empty");
    }
    
    size_t rows = controlPoints.size();
    size_t cols = controlPoints[0].size();
    
    m_weightedPoints.resize(rows);
    for (size_t i = 0; i < rows; i++) {
        m_weightedPoints[i].resize(cols);
        for (size_t j = 0; j < cols; j++) {
            m_weightedPoints[i][j] = {controlPoints[i][j], weights[i][j]};
        }
    }
}

RationalBezierSurface::RationalBezierSurface(const WeightedControlGrid& weightedPoints)
    : m_weightedPoints(weightedPoints)
{
    if (weightedPoints.empty() || weightedPoints[0].empty()) {
        throw std::invalid_argument("Control point grid cannot be empty");
    }
}

RationalBezierSurface::RationalBezierSurface(const BezierSurface& surface) {
    const auto& pts = surface.controlPoints();
    m_weightedPoints.resize(pts.size());
    
    for (size_t i = 0; i < pts.size(); i++) {
        m_weightedPoints[i].resize(pts[i].size());
        for (size_t j = 0; j < pts[i].size(); j++) {
            m_weightedPoints[i][j] = {pts[i][j], 1.0};
        }
    }
}

std::vector<std::vector<RationalBezierSurface::HomogeneousPoint4D>>
RationalBezierSurface::toHomogeneous() const 
{
    std::vector<std::vector<HomogeneousPoint4D>> result(m_weightedPoints.size());
    
    for (size_t i = 0; i < m_weightedPoints.size(); i++) {
        result[i].resize(m_weightedPoints[i].size());
        for (size_t j = 0; j < m_weightedPoints[i].size(); j++) {
            result[i][j] = HomogeneousPoint4D(
                m_weightedPoints[i][j].point,
                m_weightedPoints[i][j].weight
            );
        }
    }
    
    return result;
}

/**
 * Algorithm A1.4: 有理曲面求值
 * 
 * 在齐次空间做 de Casteljau，然后投影
 */
Point RationalBezierSurface::evaluate(double u, double v) const {
    const int n = degreeU();
    const int m = degreeV();
    
    auto homoGrid = toHomogeneous();
    
    // 第一步: 对每列做 de Casteljau(u)
    std::vector<HomogeneousPoint4D> tempPoints(m + 1);
    
    for (int j = 0; j <= m; j++) {
        std::vector<HomogeneousPoint4D> col(n + 1);
        for (int i = 0; i <= n; i++) {
            col[i] = homoGrid[i][j];
        }
        
        // de Casteljau in homogeneous space
        for (int r = 1; r <= n; r++) {
            for (int i = 0; i <= n - r; i++) {
                col[i] = HomogeneousPoint4D::lerp(col[i], col[i + 1], u);
            }
        }
        
        tempPoints[j] = col[0];
    }
    
    // 第二步: 对 tempPoints 做 de Casteljau(v)
    for (int r = 1; r <= m; r++) {
        for (int j = 0; j <= m - r; j++) {
            tempPoints[j] = HomogeneousPoint4D::lerp(tempPoints[j], tempPoints[j + 1], v);
        }
    }
    
    // 投影
    return tempPoints[0].project();
}

SurfacePoint RationalBezierSurface::evaluateFull(double u, double v) const {
    // 简化实现，只计算位置
    // TODO: 实现有理曲面的导数
    SurfacePoint result;
    result.position = evaluate(u, v);
    result.derivativeU = Point(0, 0, 0);  // 待实现
    result.derivativeV = Point(0, 0, 0);
    result.normal = Point(0, 0, 1);
    return result;
}

ControlPointGrid RationalBezierSurface::controlPointGrid() const {
    ControlPointGrid grid(m_weightedPoints.size());
    for (size_t i = 0; i < m_weightedPoints.size(); i++) {
        grid[i].resize(m_weightedPoints[i].size());
        for (size_t j = 0; j < m_weightedPoints[i].size(); j++) {
            grid[i][j] = m_weightedPoints[i][j].point;
        }
    }
    return grid;
}

std::vector<std::vector<double>> RationalBezierSurface::weightGrid() const {
    std::vector<std::vector<double>> grid(m_weightedPoints.size());
    for (size_t i = 0; i < m_weightedPoints.size(); i++) {
        grid[i].resize(m_weightedPoints[i].size());
        for (size_t j = 0; j < m_weightedPoints[i].size(); j++) {
            grid[i][j] = m_weightedPoints[i][j].weight;
        }
    }
    return grid;
}

void RationalBezierSurface::setControlPoint(int i, int j, const Point& p) {
    m_weightedPoints[i][j].point = p;
}

void RationalBezierSurface::setWeight(int i, int j, double w) {
    m_weightedPoints[i][j].weight = w;
}


//==============================================================================
// SurfaceFactory 实现
//==============================================================================

namespace SurfaceFactory 
{

BezierSurface createBilinear(const Point& p00, const Point& p10,
                              const Point& p01, const Point& p11) {
    ControlPointGrid grid = {
        {p00, p01},
        {p10, p11}
    };
    return BezierSurface(grid);
}

BezierSurface createPlane(const Point& origin,
                           const Point& axisU, const Point& axisV,
                           double sizeU, double sizeV) {
    Point p00 = origin;
    Point p10 = origin + axisU * sizeU;
    Point p01 = origin + axisV * sizeV;
    Point p11 = origin + axisU * sizeU + axisV * sizeV;
    
    return createBilinear(p00, p10, p01, p11);
}

RationalBezierSurface createCylinderPatch(const Point& center,
                                           double radius, double height,
                                           double startAngle, double endAngle) {
    // 使用圆弧 × 直线
    // 底面圆弧作为 u 方向
    auto arc = ConicFactory::createArc(Point(center.x, center.y, center.z),
                                        radius, startAngle, endAngle);
    
    // 两行: 底面和顶面
    WeightedControlGrid grid(2);
    const auto& arcPts = arc.controlPoints();
    const auto& arcW = arc.weights();
    
    grid[0].resize(arcPts.size());
    grid[1].resize(arcPts.size());
    
    for (size_t j = 0; j < arcPts.size(); j++) {
        grid[0][j] = {arcPts[j], arcW[j]};
        grid[1][j] = {Point(arcPts[j].x, arcPts[j].y, arcPts[j].z + height), arcW[j]};
    }
    
    return RationalBezierSurface(grid);
}

RationalBezierSurface createSpherePatch(const Point& center, double radius) {
    // 八分之一球: u, v 各 90°
    double w = std::sqrt(2.0) / 2.0;  // cos(45°)
    
    // 3×3 控制点网格 (二次双方向)
    WeightedControlGrid grid(3);
    for (auto& row : grid) {
        row.resize(3);
    }
    
    // 底边 (v=0): 赤道上的圆弧
    grid[0][0] = {Point(center.x + radius, center.y, center.z), 1.0};
    grid[1][0] = {Point(center.x + radius, center.y + radius, center.z), w};
    grid[2][0] = {Point(center.x, center.y + radius, center.z), 1.0};
    
    // 中间 (v=0.5): 45°纬度
    double r45 = radius * w;
    double h45 = radius * w;
    grid[0][1] = {Point(center.x + r45, center.y, center.z + h45), w};
    grid[1][1] = {Point(center.x + r45, center.y + r45, center.z + h45), w * w};
    grid[2][1] = {Point(center.x, center.y + r45, center.z + h45), w};
    
    // 顶边 (v=1): 北极
    grid[0][2] = {Point(center.x, center.y, center.z + radius), 1.0};
    grid[1][2] = {Point(center.x, center.y, center.z + radius), w};
    grid[2][2] = {Point(center.x, center.y, center.z + radius), 1.0};
    
    return RationalBezierSurface(grid);
}

RationalBezierSurface createConePatch(const Point& apex, const Point& baseCenter,
                                       double baseRadius, double startAngle, double endAngle) {
    // 锥面: 顶点到底面圆弧
    auto baseArc = ConicFactory::createArc(baseCenter, baseRadius, startAngle, endAngle);
    
    WeightedControlGrid grid(2);
    const auto& arcPts = baseArc.controlPoints();
    const auto& arcW = baseArc.weights();
    
    grid[0].resize(arcPts.size());  // 顶点行
    grid[1].resize(arcPts.size());  // 底面行
    
    for (size_t j = 0; j < arcPts.size(); j++) {
        grid[0][j] = {apex, arcW[j]};  // 顶点（权重与底面对应）
        grid[1][j] = {arcPts[j], arcW[j]};
    }
    
    return RationalBezierSurface(grid);
}

RationalBezierSurface createRevolution(const RationalBezierCurve& profile,
                                        const Point& axis,
                                        double startAngle,
                                        double endAngle) 
{
    // 旋转曲面: 轮廓曲线绕轴旋转
    // 简化实现: 假设轴是 z 轴，轮廓在 xz 平面
    
    const auto& profilePts = profile.controlPoints();
    const auto& profileW = profile.weights();
    int n = profile.degree();
    
    // 旋转方向: 使用二次有理曲线
    double theta = endAngle - startAngle;
    double w_rot = std::cos(theta / 2.0);
    
    // 3 列 (二次)，n+1 行
    WeightedControlGrid grid(n + 1);
    for (auto& row : grid) {
        row.resize(3);
    }
    
    for (int i = 0; i <= n; i++) {
        const Point& p = profilePts[i];
        double wp = profileW[i];
        double r = std::sqrt(p.x * p.x + p.y * p.y);  // 到轴的距离
        
        // 起始位置
        grid[i][0] = {Point(r * std::cos(startAngle), r * std::sin(startAngle), p.z), wp};
        
        // 中间控制点
        double midAngle = (startAngle + endAngle) / 2.0;
        double rMid = r / w_rot;
        grid[i][1] = {Point(rMid * std::cos(midAngle), rMid * std::sin(midAngle), p.z), 
                      wp * w_rot};
        
        // 结束位置
        grid[i][2] = {Point(r * std::cos(endAngle), r * std::sin(endAngle), p.z), wp};
    }
    
    return RationalBezierSurface(grid);
}

} // namespace SurfaceFactory

} // namespace nurbs
