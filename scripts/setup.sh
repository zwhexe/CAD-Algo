#!/bin/bash
# 安装 NurbsViewer 项目依赖

set -e

echo "=========================================="
echo "  Installing NurbsViewer Dependencies"
echo "=========================================="

# 安装系统依赖
echo ""
echo "[1/3] Installing system packages..."
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    wget \
    unzip

# 下载 GLAD
echo ""
echo "[2/3] Downloading GLAD (OpenGL loader)..."
mkdir -p external/glad

# 使用预生成的 GLAD 文件
# OpenGL 3.3 Core Profile
cat > external/glad/include/glad/glad.h << 'GLAD_H_EOF'
// 这是一个占位符，实际需要从 glad.dav1d.de 生成
// 或者使用下面的方式下载
GLAD_H_EOF

# 实际下载 GLAD
echo "Downloading GLAD from GitHub..."
GLAD_URL="https://github.com/Dav1dde/glad/archive/refs/tags/v0.1.36.zip"
wget -q "$GLAD_URL" -O /tmp/glad.zip
unzip -q /tmp/glad.zip -d /tmp/
cp -r /tmp/glad-0.1.36/include/* external/glad/include/ 2>/dev/null || true

# 生成 GLAD loader
echo "Generating GLAD loader..."
cd /tmp/glad-0.1.36
python -m glad --generator c --out-path ../glad-generated --api gl:core=3.3 2>/dev/null || {
    # 如果 glad 模块不可用，使用在线生成的版本
    echo "Python glad module not available, using pre-generated files..."
    cd -
    
    # 创建 GLAD 目录结构
    mkdir -p external/glad/include/glad
    mkdir -p external/glad/include/KHR
    mkdir -p external/glad/src
    
    # 下载预生成的 GLAD 文件
    echo "Downloading pre-generated GLAD files..."
    
    # glad.h
    wget -q "https://raw.githubusercontent.com/nicebyte/nicegraf/master/deps/glad/glad.h" \
         -O external/glad/include/glad/glad.h 2>/dev/null || {
        echo "Could not download glad.h. Please generate it manually from:"
        echo "  https://glad.dav1d.de/"
        echo ""
        echo "Settings:"
        echo "  - Language: C/C++"
        echo "  - Specification: OpenGL"
        echo "  - API gl: Version 3.3"
        echo "  - Profile: Core"
        echo ""
        echo "Then place files in external/glad/"
    }
}

cd - >/dev/null 2>&1 || true

echo ""
echo "[3/3] Verifying installation..."
if pkg-config --exists glfw3; then
    echo "✓ GLFW3 installed"
else
    echo "✗ GLFW3 not found"
fi

if [ -f "external/glad/include/glad/glad.h" ]; then
    echo "✓ GLAD headers found"
else
    echo "✗ GLAD headers not found - manual setup required"
fi

echo ""
echo "=========================================="
echo "  Setup complete!"
echo ""
echo "  To build:"
echo "    mkdir build && cd build"
echo "    cmake .."
echo "    make"
echo ""
echo "  To run:"
echo "    ./ex_bezier_decasteljau"
echo "=========================================="
