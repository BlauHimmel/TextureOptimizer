#ifndef OMT_HPP
#define OMT_HPP

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>

#include "Declaration.hpp"
#include "Shader.hpp"
#include "Timer.hpp"

class OMT2D
{
public:
    explicit OMT2D(TriMesh * pMesh) : m_pMesh(pMesh) { }

    bool Initialize();
    bool InitializeOpenGL();
    bool InitializeGPUBuffer();
    void Run();

public:
    /* Used for HLBFGS */
    void SetPlaneParamToGPUBuffer(const double * X);
    /* Used for HLBFGS */
    void ComputeGradientAndFunctionValFromGPUBuffer(double * G, double * F);
    /* Used for HLBFGS */
    void GLFWLoopBegin();
    /* Used for HLBFGS */
    void GLFWLoopEnd();
    /* Used for HLBFGS */
    bool GLFWShouldClose();
    /* Used for HLBFGS */
    void SetVariableFromGPUBuffer();
    /* Used for HLBFGS */
    double PARAMETER[20] = { 0.0 };
    /* Used for HLBFGS */
    int INFO[20] = { 0 };
    /* Used for HLBFGS */
    int M = 5;
    /* Used for HLBFGS */
    std::vector<double> Variable;

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

public:
    void ComputePlaneZRange()/* compute the (minZ, maxZ) for all of the planes */;
    void DrawPowerDiagramBack()/* render on the backend render target */;
    void DrawPowerDiagramScreen()/* render a quad using backend render target */;
    void ComputeCellStats()/* current area and gradient */;
    void UpdatePlane(int iterations);
    float ComputeError();
    uint32_t ComputeMissingPointNum();

private:
    TriMesh * m_pMesh = nullptr;
    GLFWwindow * m_pWindow = nullptr;

    uint32_t m_FPS = 0u;
    Timer m_FPSTimer, m_TotalTimer;
    double m_FrameCount = 0.0f;

    std::vector<float> m_TargetAreas;
    std::vector<Plane> m_InitialPlanes;
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

    const float m_Beta1 = 0.9f;
    const float m_Beta2 = 0.999f;
    const float m_Epsilon = 1e-8f;
    const float m_Alpha = 5e-5f;
    GLuint m_AdamMomentum1Buffer = GLuint(-1);
    GLuint m_AdamMomentum2Buffer = GLuint(-1);
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
