#pragma once

#include <glad/glad.h>
#include <string>

namespace viewer 
{

/**
 * OpenGL Shader 程序封装
 */
class Shader 

{
public:
    Shader(const std::string& vertexPath, const std::string& fragmentPath);
    ~Shader();
    
    // 禁止拷贝
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    
    // 使用着色器
    void use() const;
    
    // Uniform 设置
    void setFloat(const std::string& name, float value) const;
    void setVec3(const std::string& name, float x, float y, float z) const;
    void setVec4(const std::string& name, float x, float y, float z, float w) const;
    void setMat4(const std::string& name, const float* value) const;
    
    GLuint id() const { return m_program; }
    
private:
    GLuint m_program = 0;
    
    std::string loadFile(const std::string& path);
    GLuint compileShader(GLenum type, const std::string& source);
    void checkCompileErrors(GLuint shader, const std::string& type);
};

} // namespace viewer
