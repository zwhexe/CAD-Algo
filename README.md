# NURBS Viewer

基于 **The NURBS Book** 学习 CAD 曲线曲面算法的可视化项目。

## 项目结构

```
nurbs-viewer/
├── core/                    # 核心算法库
│   ├── math/               # 数学基础
│   │   └── point.hpp       # 点/向量类
│   ├── curves/             # 曲线算法
│   │   └── bezier_curve.*  # Bezier 曲线 (de Casteljau)
│   └── surfaces/           # 曲面算法 (待实现)
├── viewer/                  # 可视化模块
│   ├── window.*            # GLFW 窗口封装
│   ├── shader.*            # OpenGL 着色器
│   ├── curve_renderer.*    # 曲线渲染器
│   └── shaders/            # GLSL 着色器文件
├── examples/               # 算法示例
│   └── ex_bezier_decasteljau.cpp
├── external/               # 第三方库
│   └── glad/               # OpenGL 加载器
└── scripts/
    └── setup.sh            # 依赖安装脚本
```

## 已实现的算法

### Chapter 1: Bézier Curves

| 算法 | 书中位置 | 文件 | 示例 |
|------|----------|------|------|
| de Casteljau | Algorithm A1.1 (P24) | `bezier_curve.cpp` | `ex_bezier_decasteljau` |
| Bézier 细分 | Section 1.6 | `bezier_curve.cpp` | - |
| Bernstein 基函数 | Equation 1.8 | `bezier_curve.cpp` | - |

### Chapter 2: B-spline Basis Functions

| 算法 | 书中位置 | 文件 | 示例 |
|------|----------|------|------|
| 节点区间查找 | Algorithm A2.1 (P68) | `bspline_curve.cpp` | `ex_bspline_curve` |
| B-spline 基函数 | Algorithm A2.2 (P70) | `bspline_curve.cpp` | - |
| 非零基函数计算 | Algorithm A2.4 (P74) | `bspline_curve.cpp` | - |

### Chapter 3: B-spline Curves

| 算法 | 书中位置 | 文件 | 示例 |
|------|----------|------|------|
| de Boor 算法 | Algorithm A3.1 (P82) | `bspline_curve.cpp` | `ex_bspline_curve` |
| 节点插入 | Algorithm A5.1 (P151) | `bspline_curve.cpp` | - |

### Chapter 4: NURBS Curves

| 算法 | 书中位置 | 文件 | 示例 |
|------|----------|------|------|
| NURBS 曲线求值 | Algorithm A4.1 (P124) | `nurbs_curve.cpp` | `ex_nurbs_curve` |
| 有理基函数 | Equation 4.2 | `nurbs_curve.cpp` | - |
| 圆弧表示 | Example 7.1 (P308) | `nurbs_curve.cpp` | `ex_nurbs_curve` |
| 椭圆表示 | Section 7.3 | `nurbs_curve.cpp` | `ex_nurbs_curve` |

## 依赖

- CMake >= 3.16
- OpenGL >= 3.3
- GLFW3
- GLAD (自动配置)

## 快速开始

### Ubuntu / Debian

```bash
# 1. 安装依赖
sudo apt install cmake libglfw3-dev libgl1-mesa-dev

# 2. 配置 GLAD (从 https://glad.dav1d.de 下载)
#    - Language: C/C++
#    - gl: Version 3.3, Core profile
#    - 解压到 external/glad/

# 3. 构建
mkdir build && cd build
cmake ..
make

# 4. 运行
./ex_bezier_decasteljau
```

## 操作说明

### de Casteljau 可视化

| 操作 | 功能 |
|------|------|
| 鼠标左键拖拽 | 移动控制点 |
| 鼠标滚轮 | 调整参数 u |
| 空格 | 切换 de Casteljau 中间过程显示 |
| R | 重置到初始状态 |
| ESC | 退出 |

### 颜色说明

- **灰色** - 控制多边形
- **蓝色** - de Casteljau Level 1
- **绿色** - de Casteljau Level 2  
- **橙色** - de Casteljau Level 3
- **黑色** - Bezier 曲线
- **红色** - 曲线上当前点 C(u)

## 学习路线

按照 The NURBS Book 章节顺序：

1. **Chapter 1: Bézier Curves** ✅
   - de Casteljau Algorithm ✅
   - Bernstein Polynomials ✅
   - Curve Subdivision ✅
   
2. **Chapter 2: B-spline Basis Functions** ✅
   - Cox-de Boor Recursion ✅
   - Knot Vectors ✅
   - Basis Function Properties ✅

3. **Chapter 3: B-spline Curves** ✅
   - de Boor Algorithm ✅
   - Knot Insertion ✅
   - Curve Properties ✅

4. **Chapter 4: NURBS Curves** ✅
   - Rational Curves ✅
   - Weights ✅
   - Conic Sections ✅

5. **Chapter 5: Bézier Surfaces** (计划中)
   - Tensor Product Surfaces
   - de Casteljau for Surfaces

6. **Chapter 6: B-spline Surfaces** (计划中)
   - Surface Evaluation
   - Knot Refinement

7. **Chapter 7: NURBS Surfaces** (计划中)
   - Rational Surfaces
   - Standard Shapes

## 参考资料

- *The NURBS Book*, 2nd Edition, Piegl & Tiller
- OpenGL Programming Guide (Red Book)
- Learn OpenGL (https://learnopengl.com)

## License

MIT License - 仅供学习使用
