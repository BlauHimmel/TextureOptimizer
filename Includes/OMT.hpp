#ifndef OMT_HPP
#define OMT_HPP

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>

#include "Declaration.hpp"
#include "Shader.hpp"

class OMT2D
{
public:
    explicit OMT2D(TriMesh * pMesh) : m_pMesh(pMesh) { }

    bool Initialize();
    bool InitializeOpenGL();
    bool InitializeGPUBuffer();
    void Run(float step);

private:
    struct Color
    {
        float r, g, b;
    };

    struct Plane
    {
        float a, b, c, index;
    };

    static bool CheckInitParameterization(const TriMesh * pMesh);
    static void NormalizeUVToUnitSquare(TriMesh * pMesh);
    static void InitTargetArea(const TriMesh * pMesh, std::vector<float> & targetAreas);
    static void InitPlaneEquation(const TriMesh * pMesh, std::vector<Plane> & planes);
    static void InitCellColor(const TriMesh * pMesh, std::vector<Color> & colors);
    static void DrawQuad();

    void ComputePlaneZRange();
    void DrawPowerDiagram();
    void ComputeCellStats();
    void UpdatePlane(float step);
    float ComputeError();
    uint32_t ComputeMissingPointNum();

private:
    TriMesh * m_pMesh = nullptr;
    GLFWwindow * m_pWindow = nullptr;
    uint32_t m_FPS = 0u;

    std::vector<float> m_TargetAreas;
    std::vector<Plane> m_Planes;
    std::vector<Color> m_CellColors;

    GLuint m_TargetAreaBuffer = GLuint(-1);
    GLuint m_PlaneBuffer = GLuint(-1);
    GLuint m_CurrentAreaBuffer = GLuint(-1);
    GLuint m_GradientBuffer = GLuint(-1);
    GLuint m_CellColorBuffer = GLuint(-1);
    GLuint m_CellPixelCountBuffer = GLuint(-1);

    GLuint m_PlaneZRangeBuffer = GLuint(-1);
    GLuint m_PlaneZRangeVAO = GLuint(-1);
    std::unique_ptr<Shader> m_PlaneZRangeShader;

    GLuint m_PowerDiagramFBO = GLuint(-1);
    GLuint m_PowerDiagramDepthRBO = GLuint(-1);
    GLuint m_PowerDiagramTexture = GLuint(-1);
    GLuint m_PowerDiagramVAO = GLuint(-1);
    std::unique_ptr<Shader> m_PowerDiagramShader;
    std::unique_ptr<Shader> m_QuadShader;

    std::unique_ptr<Shader> m_CellPixelCountResetShader;
    std::unique_ptr<Shader> m_CellPixelCountShader;
    std::unique_ptr<Shader> m_CellAreaAndGradientShader;

    std::unique_ptr<Shader> m_UpdatePlaneShader;

    const int m_ClientWidth = 800;
    const int m_ClientHeight = 800;
    const int m_RenderTargetWidth = 2000;
    const int m_RenderTargetHeight = 2000;

    const float m_InitialZNear = 0.01f;
    const float m_InitialZFar= 1000.0f;
    glm::mat4 m_ModelMatrix = glm::mat4(1.0f);
    glm::mat4 m_ViewMatrix = glm::lookAt(glm::vec3(0.0f, 0.0f, 100.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    glm::mat4 m_ProjectionMatrix = glm::ortho(-1.0f, 1.0f, -1.0f, 1.0f, m_InitialZNear, m_InitialZFar);
};

#endif //OMT_HPP
