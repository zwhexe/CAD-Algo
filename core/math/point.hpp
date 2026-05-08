#pragma once

#include <cmath>
#include <iostream>
#include <vector>

namespace nurbs 
{

/**
 * 2D/3D 点和向量类
 * 用于表示控制点和曲线上的点
 */
struct Point 

{
    double x, y, z;
    
    Point(double x = 0.0, double y = 0.0, double z = 0.0) 
        : x(x), y(y), z(z) {}
    
    // 向量运算
    Point operator+(const Point& p) const {
        return {x + p.x, y + p.y, z + p.z};
    }
    
    Point operator-(const Point& p) const {
        return {x - p.x, y - p.y, z - p.z};
    }
    
    Point operator*(double t) const {
        return {x * t, y * t, z * t};
    }
    
    Point operator/(double t) const {
        return {x / t, y / t, z / t};
    }
    
    Point& operator+=(const Point& p) {
        x += p.x; y += p.y; z += p.z;
        return *this;
    }
    
    // 向量长度
    double length() const {
        return std::sqrt(x*x + y*y + z*z);
    }
    
    // 归一化
    Point normalized() const {
        double len = length();
        if (len < 1e-10) return {0, 0, 0};
        return *this / len;
    }
    
    // 点乘
    double dot(const Point& p) const {
        return x*p.x + y*p.y + z*p.z;
    }
    
    // 叉乘
    Point cross(const Point& p) const {
        return {
            y*p.z - z*p.y,
            z*p.x - x*p.z,
            x*p.y - y*p.x
        };
    }
    
    // 线性插值 - de Casteljau 的核心操作
    static Point lerp(const Point& a, const Point& b, double t) {
        return a * (1.0 - t) + b * t;
    }
    
    friend std::ostream& operator<<(std::ostream& os, const Point& p) {
        os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
        return os;
    }
};

// 方便使用的类型别名
using PointList = std::vector<Point>;

} // namespace nurbs
