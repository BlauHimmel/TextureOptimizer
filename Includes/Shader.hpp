#ifndef SHADER_H
#define SHADER_H

#include <string>
#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>

enum E_SHADER_TYPE
{
    ESHADER_TYPE_VERTEX = GL_VERTEX_SHADER,
    ESHADER_TYPE_FRAGMENT = GL_FRAGMENT_SHADER,
    ESHADER_TYPE_COMPUTE = GL_COMPUTE_SHADER,
    ESHADER_TYPE_GEOMETRY = GL_GEOMETRY_SHADER
};

class Shader
{
public:
    explicit Shader(const std::string & shaderName);
    ~Shader();

    GLuint GetProgramID() const;

    bool CompileShaderFromFile(const std::string & filename, E_SHADER_TYPE type);
    bool CompileShaderFromString(const std::string & code, E_SHADER_TYPE type);

    void LinkProgram();
    void UseProgram() const;

    void SetBool(const std::string & name, bool value) const;
    void SetInt(const std::string & name, int value) const;
    void SetFloat(const std::string & name, float value) const;
    void SetVec2(const std::string & name, const glm::vec2 & value) const;
    void SetVec2(const std::string & name, float x, float y) const;
    void SetVec3(const std::string & name, const glm::vec3 & value) const;
    void SetVec3(const std::string & name, float x, float y, float z) const;
    void SetVec4(const std::string & name, const glm::vec4 & value) const;
    void SetVec4(const std::string & name, float x, float y, float z, float w) const;
    void SetMat2(const std::string & name, const glm::mat2 & mat) const;
    void SetMat3(const std::string & name, const glm::mat3 & mat) const;
    void SetMat4(const std::string & name, const glm::mat4 & mat) const;

private:
    void CheckCompileErrors(GLuint shaderID) const;
    void CheckProgramLinkErrors(GLuint programID) const;

private:
    std::string m_ShaderName;
    GLuint m_ProgramID = 0;
    std::vector<GLuint> m_CompiledShaderIDs;
};

#endif //SHADER_H
