# The NURBS Book - Chapter 1 算法总结

本文档总结 The NURBS Book 第一章的三个核心部分，所有后续章节都建立在这些基础之上。

---

## Section 1.3: Bézier Curves (非有理 Bézier 曲线)

### 定义

**Equation 1.7** - Bézier 曲线:
```
C(u) = Σ_{i=0}^{n} B_{i,n}(u) · P_i,  u ∈ [0,1]
```

**Equation 1.8** - Bernstein 基函数:
```
B_{i,n}(u) = C(n,i) · u^i · (1-u)^{n-i}
```

其中 C(n,i) = n! / (i!(n-i)!) 是二项式系数。

### Algorithm A1.1: de Casteljau 算法 (Page 24)

这是整本书最重要的算法。用递归线性插值计算曲线上的点。

**输入**: 控制点 P_0, P_1, ..., P_n 和参数 u
**输出**: 曲线上的点 C(u)

```
初始条件: P_i^[0] = P_i

递推公式: P_i^[r] = (1-u) · P_i^[r-1] + u · P_{i+1}^[r-1]
          对于 r = 1, 2, ..., n 和 i = 0, 1, ..., n-r

最终结果: C(u) = P_0^[n]
```

**C++ 实现**:
```cpp
Point BezierCurve::evaluate(double u) const {
    const int n = degree();
    PointList Q = m_controlPoints;  // 复制
    
    for (int r = 1; r <= n; r++) {
        for (int i = 0; i <= n - r; i++) {
            Q[i] = Point::lerp(Q[i], Q[i + 1], u);  // 核心: 线性插值
        }
    }
    
    return Q[0];
}
```

**几何意义**: 每一轮将相邻的点两两连线取比例点，点数减少 1，直到剩 1 个点。

**副产品**:
1. **曲线细分**: 金字塔的左边界 {P_0^[0], P_0^[1], ..., P_0^[n]} 是 [0,u] 段的控制点
2. **导数计算**: 可以从金字塔直接提取导数信息

### 其他重要算法

**导数 (Equation 1.12)**:
```
C'(u) = n · Σ_{i=0}^{n-1} B_{i,n-1}(u) · (P_{i+1} - P_i)
```
即：n 次曲线的导数是 (n-1) 次曲线，控制点是原始控制点的一阶差分乘以 n。

**升阶 (Equation 1.17)**:
将 n 次曲线表示为 n+1 次，形状不变:
```
P_i^{new} = (i/(n+1)) · P_{i-1} + (1 - i/(n+1)) · P_i
```

---

## Section 1.4: Rational Bézier Curves (有理 Bézier 曲线)

### 定义

**Equation 1.18** - 有理 Bézier 曲线:
```
C(u) = Σ_{i=0}^{n} R_{i,n}(u) · P_i
```

**Equation 1.19** - 有理基函数:
```
R_{i,n}(u) = (B_{i,n}(u) · w_i) / Σ_{j=0}^{n}(B_{j,n}(u) · w_j)
```

其中 w_i 是控制点 P_i 的权重。

### 为什么需要有理曲线？

普通 Bézier 曲线**无法精确表示圆锥曲线**（圆、椭圆、抛物线、双曲线）。
有理 Bézier 通过引入权重，获得了这种能力。

### Algorithm A1.2: 有理 de Casteljau (Page 28)

**核心思想**: 升维 → 标准 de Casteljau → 投影

**步骤**:

1. **升维到齐次坐标** (3D → 4D):
   ```
   (P_i, w_i) → P_i^w = (w_i·x_i, w_i·y_i, w_i·z_i, w_i)
   ```

2. **在 4D 空间做标准 de Casteljau**:
   对齐次坐标点应用普通的 de Casteljau 算法

3. **透视除法** (投影回 3D):
   ```
   (X, Y, Z, W) → (X/W, Y/W, Z/W)
   ```

**C++ 实现**:
```cpp
Point RationalBezierCurve::evaluate(double u) const {
    const int n = degree();
    
    // Step 1: 转换为齐次坐标
    auto Q = toHomogeneous();  // 返回 vector<HomogeneousPoint>
    
    // Step 2: 在齐次空间做 de Casteljau
    for (int r = 1; r <= n; r++) {
        for (int i = 0; i <= n - r; i++) {
            Q[i] = HomogeneousPoint::lerp(Q[i], Q[i + 1], u);
        }
    }
    
    // Step 3: 透视除法
    return Q[0].project();  // (X/W, Y/W, Z/W)
}
```

### Section 1.4.3: 圆锥曲线

二次有理 Bézier 可以精确表示任意圆锥曲线。

**圆弧的构造**:
- 3 个控制点: P_0 (起点), P_1 (切线交点), P_2 (终点)
- 权重: w_0 = w_2 = 1, w_1 = cos(θ/2)，其中 θ 是弧度角

**90° 圆弧**: w_1 = cos(45°) = √2/2 ≈ 0.7071

---

## Section 1.5: Tensor Product Bézier Surfaces (张量积曲面)

### 定义

**Equation 1.25** - 张量积 Bézier 曲面:
```
S(u,v) = Σ_{i=0}^{n} Σ_{j=0}^{m} B_{i,n}(u) · B_{j,m}(v) · P_{i,j}
```

其中 P_{i,j} 是 (n+1) × (m+1) 的控制点网格。

### Algorithm A1.3: 曲面 de Casteljau (Page 32)

**核心思想**: 曲面 = "曲线的曲线"

**两步法**:

1. **固定 u，对每列做 de Casteljau**:
   对于 j = 0, 1, ..., m，计算 Q_j(u) = deCasteljau(P_{0,j}, P_{1,j}, ..., P_{n,j}; u)
   得到 m+1 个中间点

2. **对中间点做 de Casteljau**:
   S(u,v) = deCasteljau(Q_0, Q_1, ..., Q_m; v)

**C++ 实现**:
```cpp
Point BezierSurface::evaluate(double u, double v) const {
    const int n = degreeU();
    const int m = degreeV();
    
    // Step 1: 对每列做 de Casteljau(u)
    PointList tempPoints(m + 1);
    for (int j = 0; j <= m; j++) {
        PointList colPoints(n + 1);
        for (int i = 0; i <= n; i++) {
            colPoints[i] = m_controlPoints[i][j];
        }
        BezierCurve curve(colPoints);
        tempPoints[j] = curve.evaluate(u);
    }
    
    // Step 2: 对结果做 de Casteljau(v)
    BezierCurve finalCurve(tempPoints);
    return finalCurve.evaluate(v);
}
```

### 重要性质

1. **四角插值**: S(0,0)=P_{0,0}, S(1,0)=P_{n,0}, S(0,1)=P_{0,m}, S(1,1)=P_{n,m}

2. **边界曲线**: 四条边界都是 Bézier 曲线
   - S(u,0): 控制点 P_{i,0}
   - S(u,1): 控制点 P_{i,m}
   - S(0,v): 控制点 P_{0,j}
   - S(1,v): 控制点 P_{n,j}

3. **等参线**: 固定 u 或 v 得到的曲线是 Bézier 曲线

### Algorithm A1.4: 有理曲面 (Page 34)

与有理曲线类似，在齐次坐标空间做曲面 de Casteljau，然后投影。

---

## 三者之间的关系

```
           +weights              +v direction
Section 1.3 ────────→ Section 1.4 ────────→ Section 1.5.1
Bézier Curve       Rational Bézier    Rational Surface
                        │                    │
                        │ w_i = 1            │ w_{i,j} = 1
                        ↓                    ↓
                   Bézier Curve ──────→ Bézier Surface
                                 +v      Section 1.5
```

**核心洞察**: de Casteljau 算法是一切的基础。
- 有理曲线 = 升维 + de Casteljau + 投影
- 曲面 = 两个方向的 de Casteljau

---

## 文件对应关系

| 章节 | 算法 | 头文件 | 实现文件 |
|------|------|--------|----------|
| 1.3 | de Casteljau | bezier_curve.hpp | bezier_curve.cpp |
| 1.3 | 导数、细分、升阶 | bezier_curve.hpp | bezier_curve.cpp |
| 1.4 | Rational de Casteljau | bezier_curve.hpp | bezier_curve.cpp |
| 1.4.3 | 圆锥曲线构造 | bezier_curve.hpp (ConicFactory) | bezier_curve.cpp |
| 1.5 | Surface de Casteljau | bezier_surface.hpp | bezier_surface.cpp |
| 1.5.1 | Rational Surface | bezier_surface.hpp | bezier_surface.cpp |

---

## 学习建议

1. **首先理解 de Casteljau**: 手画几个例子，理解"递归取中点"的几何意义
2. **然后理解齐次坐标**: 这是有理曲线/曲面的关键
3. **最后理解张量积**: 把曲面看作"曲线扫出的曲线"

这三个概念掌握后，B-spline (Chapter 2-3) 和 NURBS (Chapter 4) 就是自然的推广。
