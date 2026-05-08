#include "viewer/shader.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>

namespace viewer 
{

Shader::Shader(const std::string& vertexPath, const std::string& fragmentPath) 
{
    std::string vertexSource = loadFile(vertexPath);
    std::string fragmentSource = loadFile(fragmentPath);
    
    GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexSource);
    GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentSource);
    
    // 链接程序
    m_program = glCreateProgram();
    glAttachShader(m_program, vertexShader);
    glAttachShader(m_program, fragmentShader);
    glLinkProgram(m_program);
    checkCompileErrors(m_program, "PROGRAM");
    
    // 清理
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
}

Shader::~Shader() 
{
    if (m_program) 
        glDeleteProgram(m_program);
}

void Shader::use() const 
{
    glUseProgram(m_program);
}

void Shader::setFloat(const std::string& name, float value) const 
{
    glUniform1f(glGetUniformLocation(m_program, name.c_str()), value);
}

void Shader::setVec3(const std::string& name, float x, float y, float z) const 
{
    glUniform3f(glGetUniformLocation(m_program, name.c_str()), x, y, z);
}

void Shader::setVec4(const std::string& name, float x, float y, float z, float w) const 
{
    glUniform4f(glGetUniformLocation(m_program, name.c_str()), x, y, z, w);
}

void Shader::setMat4(const std::string& name, const float* value) const 
{
    glUniformMatrix4fv(glGetUniformLocation(m_program, name.c_str()), 1, GL_FALSE, value);
}

std::string Shader::loadFile(const std::string& path) 
{
    std::ifstream file(path);
    if (!file.is_open()) 
        throw std::runtime_error("Failed to open shader file: " + path);

    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

GLuint Shader::compileShader(GLenum type, const std::string& source) 
{
    GLuint shader = glCreateShader(type);
    const char* src = source.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);
    checkCompileErrors(shader, type == GL_VERTEX_SHADER ? "VERTEX" : "FRAGMENT");
    return shader;
}

void Shader::checkCompileErrors(GLuint shader, const std::string& type) 
{
    int success;
    char infoLog[1024];
    
    if (type != "PROGRAM") 
    {
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) 
        {
            glGetShaderInfoLog(shader, 1024, nullptr, infoLog);
            throw std::runtime_error("Shader compilation error (" + type + "): " + infoLog);
        }
    } else 
    {
        glGetProgramiv(shader, GL_LINK_STATUS, &success);
        if (!success) 
        {
            glGetProgramInfoLog(shader, 1024, nullptr, infoLog);
            throw std::runtime_error("Program linking error: " + std::string(infoLog));
        }
    }
}

} // namespace viewer
