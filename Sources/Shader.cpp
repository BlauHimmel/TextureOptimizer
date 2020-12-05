#include "Shader.h"

#include <fstream>
#include <sstream>

#include "Logger.hpp"

Shader::Shader(const std::string & shaderName)
{
    m_ProgramID = glCreateProgram();
    m_ShaderName = shaderName;
}

Shader::~Shader()
{
    glDeleteProgram(m_ProgramID);
}

bool Shader::CompileShaderFromFile(const std::string & filename, E_SHADER_TYPE type)
{
    std::string shaderCode;
    std::ifstream shaderFile;
    std::stringstream stream;

    shaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    try
    {
        shaderFile.open(filename);
        stream << shaderFile.rdbuf();
        shaderFile.close();
        shaderCode = stream.str();
    }
    catch (std::ifstream::failure & ex)
    {
        Logger::Error("[%s] Shader file not not succesfully read: %s.", m_ShaderName.c_str(), ex.what());
        return false;
    }

    const char * pShaderCode = shaderCode.c_str();

    GLuint shaderID = glCreateShader(static_cast<int>(type));
    glShaderSource(shaderID, 1, &pShaderCode, nullptr);
    glCompileShader(shaderID);
    CheckCompileErrors(shaderID);
    glAttachShader(m_ProgramID, shaderID);
    m_CompiledShaderIDs.push_back(shaderID);

    return true;
}

bool Shader::CompileShaderFromString(const std::string & code, E_SHADER_TYPE type)
{
    const char * pShaderCode = code.c_str();

    GLuint shaderID = glCreateShader(static_cast<int>(type));
    glShaderSource(shaderID, 1, &pShaderCode, nullptr);
    glCompileShader(shaderID);
    CheckCompileErrors(shaderID);
    glAttachShader(m_ProgramID, shaderID);
    m_CompiledShaderIDs.push_back(shaderID);

    return true;
}

void Shader::LinkProgram()
{
    glLinkProgram(m_ProgramID);
    CheckProgramLinkErrors(m_ProgramID);

    for (const auto & shaderID : m_CompiledShaderIDs)
    {
        glDetachShader(m_ProgramID, shaderID);
        glDeleteShader(shaderID);
    }
    m_CompiledShaderIDs.clear();
    m_CompiledShaderIDs.shrink_to_fit();
}

void Shader::UseProgram() const
{
    glUseProgram(m_ProgramID);
}

void Shader::SetBool(const std::string & name, bool value) const
{
    glUniform1i(glGetUniformLocation(m_ProgramID, name.c_str()), static_cast<int>(value));
}

void Shader::SetInt(const std::string & name, int value) const
{
    glUniform1i(glGetUniformLocation(m_ProgramID, name.c_str()), value);
}

void Shader::SetFloat(const std::string & name, float value) const
{
    glUniform1f(glGetUniformLocation(m_ProgramID, name.c_str()), value);
}

void Shader::SetVec2(const std::string & name, const glm::vec2 & value) const
{
    glUniform2fv(glGetUniformLocation(m_ProgramID, name.c_str()), 1, &value[0]);
}

void Shader::SetVec2(const std::string & name, float x, float y) const
{
    glUniform2f(glGetUniformLocation(m_ProgramID, name.c_str()), x, y);
}

void Shader::SetVec3(const std::string & name, const glm::vec3 & value) const
{
    glUniform3fv(glGetUniformLocation(m_ProgramID, name.c_str()), 1, &value[0]);
}

void Shader::SetVec3(const std::string & name, float x, float y, float z) const
{
    glUniform3f(glGetUniformLocation(m_ProgramID, name.c_str()), y, y, z);
}

void Shader::SetVec4(const std::string & name, const glm::vec4 & value) const
{
    glUniform4fv(glGetUniformLocation(m_ProgramID, name.c_str()), 1, &value[0]);
}

void Shader::SetVec4(const std::string & name, float x, float y, float z, float w) const
{
    glUniform4f(glGetUniformLocation(m_ProgramID, name.c_str()), x, y, z, w);
}

void Shader::SetMat2(const std::string & name, const glm::mat2 & mat) const
{
    glUniformMatrix2fv(glGetUniformLocation(m_ProgramID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
}

void Shader::SetMat3(const std::string & name, const glm::mat3 & mat) const
{
    glUniformMatrix3fv(glGetUniformLocation(m_ProgramID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
}

void Shader::SetMat4(const std::string & name, const glm::mat4 & mat) const
{
    glUniformMatrix4fv(glGetUniformLocation(m_ProgramID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
}

void Shader::CheckCompileErrors(GLuint shaderID) const
{
    GLint success;
    GLchar logInfoBuff[1024];
    glGetShaderiv(shaderID, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(shaderID, 1024, nullptr, logInfoBuff);
        Logger::Error("[%s] Shader compilation error: %s.", m_ShaderName.c_str(), logInfoBuff);
    }
}

void Shader::CheckProgramLinkErrors(GLuint programID) const
{
    GLint success;
    GLchar logInfoBuff[1024];

    glGetProgramiv(programID, GL_LINK_STATUS, &success);
    if (!success)
    {
        glGetProgramInfoLog(programID, 1024, nullptr, logInfoBuff);
        Logger::Error("[%s] Shader program linking error: %s.", m_ShaderName.c_str(), logInfoBuff);
    }
}

