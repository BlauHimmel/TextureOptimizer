#pragma warning(disable:4067 4244)
#include "OMT.hpp"

#include <memory>
#include <random>

#include <HLBFGS.h>
#include <matplotlibcpp.h>

#include "Logger.hpp"

/*******************************************************************************************************/

const char * pComputePlaneZRangeShaderVS = R"(
#version 430 core
layout(location = 0) in vec4 plane;

out vec2 vZRange;

void main()
{
    vec2 lb = vec2(-1.0, -1.0);
    vec2 rb = vec2( 1.0, -1.0);
    vec2 lt = vec2(-1.0,  1.0);
    vec2 rt = vec2( 1.0,  1.0);

    float zlb = plane.x * lb.x + plane.y * lb.y + plane.z;
    float zrb = plane.x * rb.x + plane.y * rb.y + plane.z;
    float zlt = plane.x * lt.x + plane.y * lt.y + plane.z;
    float zrt = plane.x * rt.x + plane.y * rt.y + plane.z;

    vZRange.x = zlb;
    vZRange.x = min(vZRange.x, zrb);
    vZRange.x = min(vZRange.x, zlt);
    vZRange.x = min(vZRange.x, zrt);

    vZRange.y = zlb;
    vZRange.y = max(vZRange.y, zrb);
    vZRange.y = max(vZRange.y, zlt);
    vZRange.y = max(vZRange.y, zrt);
}
)";


/*******************************************************************************************************/

const char * pDrawPowerDiagramShaderVS = R"(
#version 430 core
layout(location = 0) in vec4 plane;
layout(location = 1) in vec3 color;

out vec4 vColor;

void main()
{
    gl_Position = vec4(plane.xyz, 1.0);
    vColor = vec4(color, plane.w);
}
)";

const char * pDrawPowerDiagramShaderGS = R"(
#version 430 core
layout(points) in ;
layout(triangle_strip, max_vertices = 4) out;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

in vec4 vColor[];

out vec4 gColor;

void main()
{
    vec3 plane = gl_in[0].gl_Position.xyz;
    vec2 lb = vec2(-1.0, -1.0);
    vec2 rb = vec2( 1.0, -1.0);
    vec2 lt = vec2(-1.0,  1.0);
    vec2 rt = vec2( 1.0,  1.0);

    gColor = vColor[0];

    mat4 mvp = projection * view * model;

    gl_Position = mvp * vec4(lb.x, lb.y, plane.x * lb.x + plane.y * lb.y + plane.z, 1.0);
    EmitVertex();
    gl_Position = mvp * vec4(rb.x, rb.y, plane.x * rb.x + plane.y * rb.y + plane.z, 1.0);
    EmitVertex();
    gl_Position = mvp * vec4(lt.x, lt.y, plane.x * lt.x + plane.y * lt.y + plane.z, 1.0);
    EmitVertex();
    gl_Position = mvp * vec4(rt.x, rt.y, plane.x * rt.x + plane.y * rt.y + plane.z, 1.0);
    EmitVertex();
	EndPrimitive();
}
)";

const char * pDrawPowerDiagramShaderFS = R"(
#version 430 core
out vec4 FragColor;

in vec4 gColor;

void main()
{
    FragColor = gColor;
}
)";

/*******************************************************************************************************/

const char * pDrawQuadShaderVS = R"(
#version 430 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec2 texcoord;

out vec2 vTexcoord;

void main()
{
    vTexcoord = texcoord;
    gl_Position = vec4(position, 1.0);
}
)";

const char * pDrawQuadShaderFS = R"(
#version 430 core
out vec4 fragColor;

in vec2 vTexcoord;
uniform sampler2D map;

void main()
{
    fragColor = texture(map, vTexcoord);
}
)";

/*******************************************************************************************************/

const char * pComputeCellPixelCountResetShaderCS = R"(
#version 430 core
layout (local_size_x = 32, local_size_y = 1, local_size_z = 1) in;
layout (std430, binding = 0) buffer cellPixelCount
{
    int pixelCount[];
};

uniform int cellNum;

void main()
{
    int x = int(gl_GlobalInvocationID.x);
    if (x < cellNum)
	{
        pixelCount[x] = 0;
    }
}
)";

const char * pComputeCellPixelCountShaderCS = R"(
#version 430 core
layout (local_size_x = 32, local_size_y = 32, local_size_z = 1) in;

layout (rgba32f, binding = 0) uniform image2D powerDiagramMap;
layout (std430, binding = 1) buffer cellPixelCount
{
    int pixelCount[];
};

uniform int powerDiagramMapWidth;
uniform int powerDiagramMapHeight;

void main()
{
    ivec2 xy = ivec2(gl_GlobalInvocationID.xy);
    if (xy.x < powerDiagramMapWidth && xy.y < powerDiagramMapHeight)
	{
        vec4 pixel = imageLoad(powerDiagramMap, xy);
        int index = int(pixel.a);
        atomicAdd(pixelCount[index], 1);
    }
}
)";

const char * pComputeCellAreaAndGradientShaderCS = R"(
#version 430 core
layout (local_size_x = 32, local_size_y = 1, local_size_z = 1) in;
layout (std430, binding = 0) buffer cellPixelCount
{
    int pixelCount[];
};
layout (std430, binding = 1) buffer currentArea
{
    float currArea[];
};
layout (std430, binding = 2) buffer targetArea
{
    float targArea[];
};
layout (std430, binding = 3) buffer gradient
{
    float grad[];
};

uniform int cellNum;
uniform int powerDiagramMapWidth;
uniform int powerDiagramMapHeight;

void main()
{
    int x = int(gl_GlobalInvocationID.x);
    if (x < cellNum)
	{
        float totalTargetArea = 4.0;
        currArea[x] = float(pixelCount[x]) / float(powerDiagramMapWidth * powerDiagramMapHeight) * totalTargetArea;
        grad[x] = currArea[x] - targArea[x];
    }
}
)";

/*******************************************************************************************************/

const char * pUpdatePlaneShaderCS = R"(
#version 430 core
layout (local_size_x = 32, local_size_y = 1, local_size_z = 1) in;
layout (std430, binding = 0) buffer plane
{
    vec4 p[];
};
layout (std430, binding = 1) buffer gradient
{
    float grad[];
};
layout (std430, binding = 2) buffer adamMomentum1
{
    float momentum1[];
};
layout (std430, binding = 3) buffer adamMomentum2
{
    float momentum2[];
};

uniform int cellNum;
uniform float t;
uniform float beta1;
uniform float beta2;
uniform float alpha;

void main()
{
    int x = int(gl_GlobalInvocationID.x);
    if (x < cellNum)
	{
        momentum1[x] = beta1 * momentum1[x] + (1.0 - beta1) * grad[x];
        momentum2[x] = beta2 * momentum2[x] + (1.0 - beta2) * grad[x] * grad[x];

        float momentum1Hat = momentum1[x] / (1.0 - pow(beta1, t));

        float rhoLimit = 2.0 / (1.0 - beta2) - 1.0;
        float beta2t = pow(beta2, t);
        float rho = rhoLimit - 2.0 * t * beta2t / (1.0 - beta2t);
        if (rho > 4.0)
        {
            float momentum2Hat = sqrt(momentum2[x]) / (1.0 - beta2t);
            float rectification = sqrt(((rho - 4.0) * (rho - 2.0) * rhoLimit) / ((rhoLimit - 4.0) * (rhoLimit - 2.0) * rho));
            float u = rectification * momentum1Hat / momentum2Hat;
            p[x].z -= (u * alpha);
        }
        else
        {
            float u = momentum1Hat;
            p[x].z -= (u * alpha);
        }
    }
}
)";

/*******************************************************************************************************/

const char * pComputeCellSiteResetShaderCS = R"(
#version 430 core
layout (local_size_x = 32, local_size_y = 1, local_size_z = 1) in;
layout (std430, binding = 0) buffer cellPositionAccumulate
{
    ivec2 accu[];
};

uniform int cellNum;

void main()
{
    int x = int(gl_GlobalInvocationID.x);
    if (x < cellNum)
	{
        accu[x].x = 0;
        accu[x].y = 0;
    }
}
)";

const char * pComputeCellSiteShaderCS = R"(
#version 430 core
layout (local_size_x = 32, local_size_y = 32, local_size_z = 1) in;

layout (rgba32f, binding = 0) uniform image2D powerDiagramMap;
layout (std430, binding = 1) buffer cellPixelCount
{
    int pixelCount[];
};
layout (std430, binding = 2) buffer cellPositionAccumulate
{
    ivec2 accu[];
};

uniform int powerDiagramMapWidth;
uniform int powerDiagramMapHeight;

void main()
{
    ivec2 xy = ivec2(gl_GlobalInvocationID.xy);
    if (xy.x < powerDiagramMapWidth && xy.y < powerDiagramMapHeight)
	{
        vec4 pixel = imageLoad(powerDiagramMap, xy);
        int index = int(pixel.a);
        atomicAdd(pixelCount[index], 1);
        atomicAdd(accu[index].x, xy.x);
        atomicAdd(accu[index].y, xy.y);
    }
}
)";

/*******************************************************************************************************/

/* Used for HLBFGS */
OMT2D * g_pGlobalPointer = nullptr;
double g_Threshold = 0.0; /* This value is modfied each time Run() is called. When Run() is finished, this value is reset to 0.0. */
bool g_bThresholdReached = false;

/* Used for HLBFGS */
void EVALFUNC([[maybe_unused]] int N, double * X, [[maybe_unused]] double * PREV_X, double * F, double * G)
{
    g_pGlobalPointer->SetPlaneParamToGPUBuffer(X);
    g_pGlobalPointer->ComputePlaneZRange();
    g_pGlobalPointer->DrawPowerDiagramBack();
    g_pGlobalPointer->ComputeCellStats();
    g_pGlobalPointer->ComputeGradientAndFunctionValFromGPUBuffer(G, F);
}

/* Used for HLBFGS */
void NEWITERATION(int ITER, [[maybe_unused]] int CALL_ITER, [[maybe_unused]] double * X, [[maybe_unused]] double * F, [[maybe_unused]] double * G, [[maybe_unused]] double * GNORM)
{
    g_pGlobalPointer->GLFWLoopBegin(true, false);
    g_pGlobalPointer->DrawPowerDiagramScreen();
    g_pGlobalPointer->GLFWLoopEnd();

    double error = g_pGlobalPointer->ComputeError();
    uint32_t missingPointNum = g_pGlobalPointer->ComputeMissingPointNum();
    Logger::Info("HLBFGS: Iterations: %d, Error = %.8f, Missing Point = %u, Elapsed: %s.",
                 ITER - 1, error, missingPointNum, g_pGlobalPointer->GetElapsedTimeString().c_str());

    if (error <= g_Threshold)
    {
        /* Stop the iteration manually. */
        g_bThresholdReached = true;
        g_pGlobalPointer->INFO[4] = g_pGlobalPointer->INFO[2] - 1;
        return;
    }

    if (g_pGlobalPointer->GLFWShouldClose())
    {
        /* Stop the iteration manually. */
        g_pGlobalPointer->INFO[4] = g_pGlobalPointer->INFO[2] - 1;
        return;
    }
}

/*******************************************************************************************************/

bool OMT2D::Initialize(int maxIterationsHLBFGS)
{
    if (!InitializeOpenGL())
    {
        Logger::Error("Failed to initialize OpenGL.");
        return false;
    }
    if (!InitializeGPUBuffer())
    {
        Logger::Error("Failed to initialize GPU buffers.");
        return false;
    }

    /* Init HLBFGS parameters. */
    INIT_HLBFGS(PARAMETER, INFO);
    /* function tolerance used in line-search. */
    PARAMETER[0] = 1.0e-12;
    /* variable tolerance used in line-search. */
    PARAMETER[1] = 1.0e-16;
    /* gradient tolerance used in line-search. */
    PARAMETER[2] = 0.9999;
    /* Minimum step. */
    PARAMETER[3] = 1.0e-20;
    /* Maximum step. */
    PARAMETER[4] = 1.0e20;
    /* The stop criterion ( ||G||/max(1,||X||) < PARAMETER[5] */
    PARAMETER[5] = 0.0;
    /* The stop criterion ( ||G|| < PARAMETER[6] ) */
    PARAMETER[6] = 0.0;
    /* the max number of evaluation in line-search */
    INFO[0] = 20;
    /* The LBFGS strategy. 0: standard, 1 : M1QN3 strategy[8](recommended). */
    INFO[3] = 1;
    /* The max number of iterations */
    INFO[4] = maxIterationsHLBFGS;
    /* 1: print message, 0: do nothing */
    INFO[5] = 0;
    /* The update interval of Hessian. (typical choices : 0 - 200) */
    INFO[6] = 0;
    /* 0 : without hessian, 1 : with accurate hessian */
    INFO[7] = 0;
    /* 0: Disable preconditioned CG; 1: Enable preconditioned CG */
    INFO[10] = 0;

    return true;
}

bool OMT2D::InitializeOpenGL()
{
    if (glfwInit() != GLFW_TRUE)
    {
        Logger::Error("Failed to initialize GLFW.");
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    m_pWindow = glfwCreateWindow(m_ClientWidth, m_ClientHeight, "OMT", nullptr, nullptr);

    if (m_pWindow == nullptr)
    {
        Logger::Error("Failed to create GLFW window.");
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_pWindow);
    glfwSwapInterval(0);

    glfwSetWindowAttrib(m_pWindow, GLFW_RESIZABLE, GLFW_FALSE);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        Logger::Error("Failed to initialize GLAD.");
        glfwTerminate();
        return false;
    }

    return true;
}

bool OMT2D::InitializeGPUBuffer()
{
    if (!CheckInitParameterization(m_pMesh)) { return false; }
    NormalizeUVToUnitSquare(m_pMesh);

    /*******************************************************************************************************/

    if (m_TargetAreaBuffer != GLuint(-1)) { glDeleteBuffers(1, &m_TargetAreaBuffer); }
    glGenBuffers(1, &m_TargetAreaBuffer);
    if (!m_bTargetAreaAssignedWhenConstruct)
    {
        InitTargetArea(m_pMesh, m_TargetAreas);
    }
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_TargetAreaBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, m_TargetAreas.size() * sizeof(float), m_TargetAreas.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    if (m_PlaneBuffer != GLuint(-1)) { glDeleteBuffers(1, &m_PlaneBuffer); }
    glGenBuffers(1, &m_PlaneBuffer);
    InitPlaneEquation(m_pMesh, m_InitialPlanes);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_PlaneBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, m_InitialPlanes.size() * sizeof(Plane), m_InitialPlanes.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    std::vector<float> zeroBufferFloat(m_pMesh->n_vertices(), 0.0f);

    if (m_CurrentAreaBuffer != GLuint(-1)) { glDeleteBuffers(1, &m_CurrentAreaBuffer); }
    glGenBuffers(1, &m_CurrentAreaBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_CurrentAreaBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, zeroBufferFloat.size() * sizeof(float), zeroBufferFloat.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    if (m_GradientBuffer != GLuint(-1)) { glDeleteBuffers(1, &m_GradientBuffer); }
    glGenBuffers(1, &m_GradientBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_GradientBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, zeroBufferFloat.size() * sizeof(float), zeroBufferFloat.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    if (m_CellColorBuffer != GLuint(-1)) { glDeleteBuffers(1, &m_CellColorBuffer); }
    glGenBuffers(1, &m_CellColorBuffer);
    InitCellColor(m_pMesh, m_CellColors);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_CellColorBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, m_CellColors.size() * sizeof(Color), m_CellColors.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    std::vector<int> zeroBufferInt(m_pMesh->n_vertices(), 0);

    if (m_CellPixelCountBuffer != GLuint(-1)) { glDeleteBuffers(1, &m_CellPixelCountBuffer); }
    glGenBuffers(1, &m_CellPixelCountBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_CellPixelCountBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, zeroBufferInt.size() * sizeof(int), zeroBufferInt.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    /*******************************************************************************************************/

    std::vector<float> zeroBufferFloat2(m_pMesh->n_vertices() * 2, 0.0f);

    if (m_PlaneZRangeBuffer != GLuint(-1)) { glDeleteBuffers(1, &m_PlaneZRangeBuffer); }
    glGenBuffers(1, &m_PlaneZRangeBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_PlaneZRangeBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, zeroBufferFloat2.size() * sizeof(float), zeroBufferFloat2.data(), GL_STATIC_READ);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    if (m_PlaneZRangeVAO != GLuint(-1)) { glDeleteVertexArrays(1, &m_PlaneZRangeVAO); }
    glGenVertexArrays(1, &m_PlaneZRangeVAO);
    glBindVertexArray(m_PlaneZRangeVAO);

    glBindBuffer(GL_ARRAY_BUFFER, m_PlaneBuffer);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(Plane), nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    m_PlaneZRangeShader = std::make_unique<Shader>("Plane Z Range Shader");
    m_PlaneZRangeShader->CompileShaderFromString(pComputePlaneZRangeShaderVS, ESHADER_TYPE_VERTEX);
    const GLchar * pFeedbackVaryings[] = { "vZRange" };
    glTransformFeedbackVaryings(m_PlaneZRangeShader->GetProgramID(), 1, pFeedbackVaryings, GL_INTERLEAVED_ATTRIBS);
    m_PlaneZRangeShader->LinkProgram();

    /*******************************************************************************************************/

    if (m_PowerDiagramTexture != GLuint(-1)) { glDeleteTextures(1, &m_PowerDiagramTexture); }
    glGenTextures(1, &m_PowerDiagramTexture);
    glBindTexture(GL_TEXTURE_2D, m_PowerDiagramTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, m_RenderTargetWidth, m_RenderTargetHeight, 0, GL_RGBA, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    if (m_PowerDiagramDepthRBO != GLuint(-1)) { glDeleteRenderbuffers(1, &m_PowerDiagramDepthRBO); }
    glGenRenderbuffers(1, &m_PowerDiagramDepthRBO);
    glBindRenderbuffer(GL_RENDERBUFFER, m_PowerDiagramDepthRBO);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32, m_RenderTargetWidth, m_RenderTargetHeight);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    if (m_PowerDiagramFBO != GLuint(-1)) { glDeleteFramebuffers(1, &m_PowerDiagramFBO); }
    glGenFramebuffers(1, &m_PowerDiagramFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, m_PowerDiagramFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_PowerDiagramTexture, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_PowerDiagramDepthRBO);
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        Logger::Error("ERROR::FRAMEBUFFER:: Framebuffer is not complete!");
        return false;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    if (m_PowerDiagramVAO != GLuint(-1)) { glDeleteVertexArrays(1, &m_PowerDiagramVAO); }
    glGenVertexArrays(1, &m_PowerDiagramVAO);
    glBindVertexArray(m_PowerDiagramVAO);

    glBindBuffer(GL_ARRAY_BUFFER, m_PlaneBuffer);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(Plane), nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, m_CellColorBuffer);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Color), nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    m_PowerDiagramShader = std::make_unique<Shader>("Power Diagram Shader");
    m_PowerDiagramShader->CompileShaderFromString(pDrawPowerDiagramShaderVS, ESHADER_TYPE_VERTEX);
    m_PowerDiagramShader->CompileShaderFromString(pDrawPowerDiagramShaderGS, ESHADER_TYPE_GEOMETRY);
    m_PowerDiagramShader->CompileShaderFromString(pDrawPowerDiagramShaderFS, ESHADER_TYPE_FRAGMENT);
    m_PowerDiagramShader->LinkProgram();

    m_QuadShader = std::make_unique<Shader>("Quad Shader");
    m_QuadShader->CompileShaderFromString(pDrawQuadShaderVS, ESHADER_TYPE_VERTEX);
    m_QuadShader->CompileShaderFromString(pDrawQuadShaderFS, ESHADER_TYPE_FRAGMENT);
    m_QuadShader->LinkProgram();

    /*******************************************************************************************************/

    m_CellPixelCountResetShader = std::make_unique<Shader>("Cell Pixel Count Reset Shader");
    m_CellPixelCountResetShader->CompileShaderFromString(pComputeCellPixelCountResetShaderCS, ESHADER_TYPE_COMPUTE);
    m_CellPixelCountResetShader->LinkProgram();

    m_CellPixelCountShader = std::make_unique<Shader>("Cell Pixel Count Shader");
    m_CellPixelCountShader->CompileShaderFromString(pComputeCellPixelCountShaderCS, ESHADER_TYPE_COMPUTE);
    m_CellPixelCountShader->LinkProgram();

    m_CellAreaAndGradientShader = std::make_unique<Shader>("Cell Area And Gradient Shader");
    m_CellAreaAndGradientShader->CompileShaderFromString(pComputeCellAreaAndGradientShaderCS, ESHADER_TYPE_COMPUTE);
    m_CellAreaAndGradientShader->LinkProgram();

    /*******************************************************************************************************/

    if (m_AdamMomentum1Buffer != GLuint(-1)) { glDeleteBuffers(1, &m_AdamMomentum1Buffer); }
    glGenBuffers(1, &m_AdamMomentum1Buffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_AdamMomentum1Buffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, zeroBufferFloat.size() * sizeof(float), zeroBufferFloat.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    if (m_AdamMomentum2Buffer != GLuint(-1)) { glDeleteBuffers(1, &m_AdamMomentum2Buffer); }
    glGenBuffers(1, &m_AdamMomentum2Buffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_AdamMomentum2Buffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, zeroBufferFloat.size() * sizeof(float), zeroBufferFloat.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    m_UpdatePlaneShader = std::make_unique<Shader>("Update Plane Shader");
    m_UpdatePlaneShader->CompileShaderFromString(pUpdatePlaneShaderCS, ESHADER_TYPE_COMPUTE);
    m_UpdatePlaneShader->LinkProgram();

    /*******************************************************************************************************/

    std::vector<int> zeroBufferInt2(m_pMesh->n_vertices() * 2, 0);

    if (m_CellPositionAccumulateBuffer != GLuint(-1)) { glDeleteBuffers(1, &m_CellPositionAccumulateBuffer); }
    glGenBuffers(1, &m_CellPositionAccumulateBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_CellPositionAccumulateBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, zeroBufferInt2.size() * sizeof(int), zeroBufferInt2.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    m_ComputeCellSiteResetShader = std::make_unique<Shader>("Compute Cell Site Reset Shader");
    m_ComputeCellSiteResetShader->CompileShaderFromString(pComputeCellSiteResetShaderCS, ESHADER_TYPE_COMPUTE);
    m_ComputeCellSiteResetShader->LinkProgram();

    m_ComputeCellSiteShader = std::make_unique<Shader>("Compute Cell Site Shader");
    m_ComputeCellSiteShader->CompileShaderFromString(pComputeCellSiteShaderCS, ESHADER_TYPE_COMPUTE);
    m_ComputeCellSiteShader->LinkProgram();

    /*******************************************************************************************************/

    return true;
}

void OMT2D::Run(double threshold)
{
    if (g_pGlobalPointer != nullptr)
    {
        Logger::Error("Multiple OMT2D class running at the same time!");
        return;
    }

    g_pGlobalPointer = this;
    g_Threshold = threshold;
    g_bThresholdReached = false;

    m_FPSTimer.Reset();
    m_TotalTimer.Reset();
    m_FrameCount = 0.0;

    m_ErrorsHLBFGS.clear();
    m_ErrorsGradientDescent.clear();
    m_MissingPointHLBFGS.clear();
    m_MissingPointGradientDescent.clear();

    SetVariableFromGPUBuffer();
    HLBFGS(int(m_pMesh->n_vertices()), M, Variable.data(), EVALFUNC, nullptr, HLBFGS_UPDATE_Hessian, NEWITERATION, PARAMETER, INFO);

    int32_t iterations = 1;
    while (!glfwWindowShouldClose(m_pWindow) && !g_bThresholdReached)
    {
        GLFWLoopBegin(false, true);
        ComputePlaneZRange();
        DrawPowerDiagramBack();
        DrawPowerDiagramScreen();
        ComputeCellStats();
        UpdatePlane(iterations++);
        GLFWLoopEnd();
    }

    if (g_bThresholdReached)
    {
        Logger::Info("Threshold reached, stop the iterations.");
    }
    else
    {
        Logger::Info("User manually stop the iterations.");
    }

    Logger::Info("Applying optimal mass transport map...");
    ComputeCellSite();
    Logger::Info("Apply completed.");

    glfwTerminate();

    g_pGlobalPointer = nullptr;
    g_Threshold = 0.0;
    g_bThresholdReached = false;
}

void OMT2D::SetPlaneParamToGPUBuffer(const double * X)
{
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_PlaneBuffer);
    auto * pPlane = reinterpret_cast<Plane *>(glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_WRITE));
    for (size_t i = 0; i < m_pMesh->n_vertices(); i++)
    {
        pPlane[i].c = static_cast<float>(X[i]);
    }
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT);
}

void OMT2D::ComputeGradientAndFunctionValFromGPUBuffer(double * G, double * F)
{
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_GradientBuffer);
    auto * pGradient = reinterpret_cast<float *>(glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY));
    for (size_t i = 0; i < m_pMesh->n_vertices(); i++)
    {
        G[i] = static_cast<double>(pGradient[i]);
        *F += (G[i] * G[i]);
    }
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    static std::unique_ptr<float[]> buffer(new float[m_RenderTargetWidth * m_RenderTargetHeight * 4]);
    memset(buffer.get(), 0, 4 * m_RenderTargetWidth * m_RenderTargetHeight * sizeof(float));

    glBindTexture(GL_TEXTURE_2D, m_PowerDiagramTexture);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, buffer.get());

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_PlaneBuffer);
    auto * pPlane = reinterpret_cast<Plane *>(glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY));

    float pixelArea = (2.0f / static_cast<float>(m_RenderTargetWidth)) * (2.0f / static_cast<float>(m_RenderTargetHeight));
    *F = 0.0;
    for (int y = 0; y < m_RenderTargetHeight; y++)
    {
        for (int x = 0; x < m_RenderTargetWidth; x++)
        {
            auto index = static_cast<int>(buffer[(y * m_RenderTargetWidth + x) * 4 + 3]);
            float xx = (static_cast<float>(x) / static_cast<float>(m_RenderTargetWidth) * 2.0f - 1.0f);
            float yy = (static_cast<float>(y) / static_cast<float>(m_RenderTargetHeight) * 2.0f - 1.0f);
            float zz = xx * pPlane[index].a + yy * pPlane[index].b + pPlane[index].c;
            float vol = pixelArea * zz;
            *F += vol;
        }
    }

    for (size_t i = 0; i < m_pMesh->n_vertices(); i++)
    {
        *F -= (m_TargetAreas[i] * pPlane[i].c);
    }

    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void OMT2D::GLFWLoopBegin(bool bStoreHLBFGSInfo, bool bStoreGradientDescentInfo)
{
    m_FrameCount += 1.0;
    if (m_FPSTimer.Elapsed() >= 1000.0)
    {
        m_FPS = static_cast<uint32_t>(m_FrameCount * 1000.0 / m_FPSTimer.Elapsed());
        m_FrameCount = 0.0;
        m_FPSTimer.Reset();

        float error = ComputeError();
        uint32_t missingPointNum = ComputeMissingPointNum();

        std::string title = Logger::Format("OMT %u Iter./Sec, Err. = %.8f , Miss.: %u, Elap.: %s. (Close the window to stop the iteration)",
            m_FPS, error, missingPointNum, m_TotalTimer.ElapsedString(true).c_str());
        glfwSetWindowTitle(m_pWindow, title.c_str());

        if (bStoreGradientDescentInfo)
        {
            m_ErrorsGradientDescent.emplace_back(GetElapsedTime(), error);
            m_MissingPointGradientDescent.emplace_back(GetElapsedTime(), missingPointNum);
        }

        if (bStoreHLBFGSInfo)
        {
            m_ErrorsHLBFGS.emplace_back(GetElapsedTime(), error);
            m_MissingPointHLBFGS.emplace_back(GetElapsedTime(), missingPointNum);
        }

        if (error <= g_Threshold)
        {
            g_bThresholdReached = true;
        }
    }
}

void OMT2D::GLFWLoopEnd()
{
    glfwSwapBuffers(m_pWindow);
    glfwPollEvents();
}

bool OMT2D::GLFWShouldClose()
{
    return glfwWindowShouldClose(m_pWindow);
}

void OMT2D::SetVariableFromGPUBuffer()
{
    Variable.resize(m_pMesh->n_vertices(), 0.0);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_PlaneBuffer);
    auto * pPlane = reinterpret_cast<Plane *>(glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY));
    for (size_t i = 0; i < m_pMesh->n_vertices(); i++)
    {
        Variable[i] = pPlane[i].c;
    }
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

double OMT2D::GetElapsedTime()
{
    return m_TotalTimer.Elapsed();
}

std::string OMT2D::GetElapsedTimeString()
{
    return m_TotalTimer.ElapsedString(true);
}

bool OMT2D::CheckInitParameterization(const TriMesh * pMesh)
{
    if (!pMesh->has_vertex_texcoords2D())
    {
        Logger::Error("No initial parameterization found.");
        return false;
    }

    std::set<std::pair<Real, Real>> uvSet;
    bool bOverlapped = false;
    for (const auto & vh : pMesh->vertices())
    {
        const auto & uv = pMesh->texcoord2D(vh);
        auto [iter, bSuccess] = uvSet.insert({ uv[0], uv[1] });
        if (!bSuccess)
        {
            Logger::Error("Overlapped uv texcoord found. index = %d, uv = (%lf, %lf).", vh.idx(), uv[0], uv[1]);
            bOverlapped = true;
        }
    }

    return !bOverlapped;
}

void OMT2D::NormalizeUVToUnitSquare(TriMesh * pMesh)
{
    Real uMax = -std::numeric_limits<Real>::max();
    Real vMax = -std::numeric_limits<Real>::max();
    Real uMin =  std::numeric_limits<Real>::max();
    Real vMin =  std::numeric_limits<Real>::max();

    for (const auto & vh : pMesh->vertices())
    {
        const auto & uv = pMesh->texcoord2D(vh);
        uMax = std::max(uMax, uv[0]);
        vMax = std::max(vMax, uv[1]);
        uMin = std::min(uMin, uv[0]);
        vMin = std::min(vMin, uv[1]);
    }

    Real invDeltaU = 1.0 / (uMax - uMin);
    Real invDeltaV = 1.0 / (vMax - vMin);

    for (auto & vh : pMesh->vertices())
    {
        const auto & uv = pMesh->texcoord2D(vh);
        pMesh->set_texcoord2D(vh, {
            (uv[0] - uMin) * invDeltaU * 2.0 - 1.0,
            (uv[1] - vMin) * invDeltaV * 2.0 - 1.0
        });
    }
}

void OMT2D::InitTargetArea(const TriMesh * pMesh, std::vector<float> & targetAreas)
{
    targetAreas.resize(pMesh->n_vertices(), 0.0f);

    Real totalMeshArea = 0.0f;
    for (const auto & fh : pMesh->faces())
    {
        TriMesh::Point points[3];
        size_t index = 0;

        for (const auto & vh : pMesh->fv_range(fh))
        {
            points[index++] = pMesh->point(vh);
        }

        const auto & va = points[0];
        const auto & vb = points[1];
        const auto & vc = points[2];

        auto lba = va - vb;
        auto lbc = vc - vb;

        Real area = lba.cross(lbc).norm() * 0.5;
        totalMeshArea += area;
    }

    for (const auto & vh : pMesh->vertices())
    {
        Real oneRingMeshArea = 0.0;
        for (const auto & fh : pMesh->vf_range(vh))
        {
            TriMesh::Point points[3];
            size_t index = 0;

            for (const auto & fhvh : pMesh->fv_range(fh))
            {
                points[index++] = pMesh->point(fhvh);
            }

            const auto & va = points[0];
            const auto & vb = points[1];
            const auto & vc = points[2];

            auto lba = va - vb;
            auto lbc = vc - vb;

            Real area = lba.cross(lbc).norm() * 0.5;
            oneRingMeshArea += area / 3.0;
        }

        const Real totalTargetArea = 4.0;
        targetAreas[vh.idx()] = static_cast<float>(oneRingMeshArea / totalMeshArea * totalTargetArea);
    }
}

void OMT2D::InitPlaneEquation(const TriMesh * pMesh, std::vector<Plane> & planes)
{
    planes.resize(pMesh->n_vertices(),{ 0.0f, 0.0f, 0.0f });

    for (const auto & vh : pMesh->vertices())
    {
        const auto & uv = pMesh->texcoord2D(vh);
        planes[vh.idx()].a = static_cast<float>(uv[0]);
        planes[vh.idx()].b = static_cast<float>(uv[1]);
        planes[vh.idx()].c = static_cast<float>(-0.5 * uv.squaredNorm());
        planes[vh.idx()].index = static_cast<float>(vh.idx());
    }
}

void OMT2D::InitCellColor(const TriMesh * pMesh, std::vector<Color> & colors)
{
    colors.resize(pMesh->n_vertices(),{ 0.0f, 0.0f, 0.0f });

    std::default_random_engine engine(static_cast<unsigned int>(pMesh->n_vertices()));
    std::uniform_real_distribution<float> uniform(0.0, 1.0);
    for (size_t i = 0; i < pMesh->n_vertices(); i++)
    {
        std::vector<float> color = { uniform(engine), uniform(engine), uniform(engine) };
        colors[i].r = uniform(engine);
        colors[i].g = uniform(engine);
        colors[i].b = uniform(engine);
    }
}

void OMT2D::DrawQuad()
{
    static auto quadVAO = GLuint(-1);
    static auto quadVBO = GLuint(-1);

    static float quadVertices[]
    {
        -1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
        -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
         1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
         1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
    };

    if (quadVAO == GLuint(-1) && quadVBO == GLuint(-1))
    {
        glGenVertexArrays(1, &quadVAO);
        glGenBuffers(1, &quadVBO);

        glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);

        glBindVertexArray(quadVAO);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), nullptr);

        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
#if _DEBUG
        Logger::Error("Draw Quad: Framebuffer is not complete!");
#endif
        return;
    }

    glBindVertexArray(quadVAO);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);
}

void OMT2D::ComputePlaneZRange()
{
    m_PlaneZRangeShader->UseProgram();

    glEnable(GL_RASTERIZER_DISCARD);
    glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, m_PlaneZRangeBuffer);
    glBeginTransformFeedback(GL_POINTS);
    glBindVertexArray(m_PlaneZRangeVAO);
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(m_pMesh->n_vertices()));
    glBindVertexArray(0);
    glEndTransformFeedback();
    glDisable(GL_RASTERIZER_DISCARD);
    glFlush();
    glMemoryBarrier(GL_TRANSFORM_FEEDBACK_BARRIER_BIT);

    glBindBuffer(GL_TRANSFORM_FEEDBACK_BUFFER, m_PlaneZRangeBuffer);
    auto * pRange = reinterpret_cast<float *>(glMapBuffer(GL_TRANSFORM_FEEDBACK_BUFFER, GL_READ_ONLY));
    float zMin =  std::numeric_limits<float>::max();
    float zMax = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < m_pMesh->n_vertices() * 2; i += 2)
    {
        zMin = std::min(zMin, pRange[i + 0]);
        zMax = std::max(zMax, pRange[i + 1]);
    }
    glUnmapBuffer(GL_TRANSFORM_FEEDBACK_BUFFER);
    glBindBuffer(GL_TRANSFORM_FEEDBACK_BUFFER, 0);

    m_ViewMatrix = glm::lookAt(glm::vec3(0.0f, 0.0f, zMax), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    m_ProjectionMatrix = glm::ortho(-1.0f, 1.0f, -1.0f, 1.0f, 0.0f, zMax - zMin);
}

void OMT2D::DrawPowerDiagramBack()
{
    glEnable(GL_DEPTH_TEST);

    glBindFramebuffer(GL_FRAMEBUFFER, m_PowerDiagramFBO);
    m_PowerDiagramShader->UseProgram();
    m_PowerDiagramShader->SetMat4("model", m_ModelMatrix);
    m_PowerDiagramShader->SetMat4("view", m_ViewMatrix);
    m_PowerDiagramShader->SetMat4("projection", m_ProjectionMatrix);
    glViewport(0, 0, m_RenderTargetWidth, m_RenderTargetHeight);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glBindVertexArray(m_PowerDiagramVAO);
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(m_pMesh->n_vertices()));
    glBindVertexArray(0);
}

void OMT2D::DrawPowerDiagramScreen()
{
    glEnable(GL_DEPTH_TEST);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    m_QuadShader->UseProgram();
    m_QuadShader->SetInt("map", 0);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, m_ClientWidth, m_ClientHeight);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_PowerDiagramTexture);
    DrawQuad();
}

void OMT2D::ComputeCellStats()
{
    m_CellPixelCountResetShader->UseProgram();
    m_CellPixelCountResetShader->SetInt("cellNum", static_cast<int>(m_pMesh->n_vertices()));
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_CellPixelCountBuffer);
    glDispatchCompute(static_cast<GLuint>(m_pMesh->n_vertices() / 32 + 1), 1u, 1u);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    m_CellPixelCountShader->UseProgram();
    m_CellPixelCountShader->SetInt("powerDiagramMapWidth", m_RenderTargetWidth);
    m_CellPixelCountShader->SetInt("powerDiagramMapHeight", m_RenderTargetHeight);
    m_CellPixelCountShader->SetInt("powerDiagramMap", 0);
    glBindImageTexture(0, m_PowerDiagramTexture, 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, m_CellPixelCountBuffer);
    glDispatchCompute(static_cast<GLuint>(m_RenderTargetWidth / 32 + 1), static_cast<GLuint>(m_RenderTargetHeight / 32 + 1), 1u);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    m_CellAreaAndGradientShader->UseProgram();
    m_CellAreaAndGradientShader->SetInt("cellNum", static_cast<int>(m_pMesh->n_vertices()));
    m_CellAreaAndGradientShader->SetInt("powerDiagramMapWidth", m_RenderTargetWidth);
    m_CellAreaAndGradientShader->SetInt("powerDiagramMapHeight", m_RenderTargetHeight);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_CellPixelCountBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, m_CurrentAreaBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_TargetAreaBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, m_GradientBuffer);
    glDispatchCompute(static_cast<GLuint>(m_pMesh->n_vertices() / 32 + 1), 1u, 1u);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void OMT2D::UpdatePlane(int iterations)
{
    m_UpdatePlaneShader->UseProgram();
    m_UpdatePlaneShader->SetInt("cellNum", static_cast<int>(m_pMesh->n_vertices()));
    m_UpdatePlaneShader->SetFloat("t", static_cast<float>(iterations));
    m_UpdatePlaneShader->SetFloat("beta1", m_Beta1);
    m_UpdatePlaneShader->SetFloat("beta2", m_Beta2);
    m_UpdatePlaneShader->SetFloat("alpha", m_Alpha);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_PlaneBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, m_GradientBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_AdamMomentum1Buffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, m_AdamMomentum2Buffer);
    glDispatchCompute(static_cast<GLuint>(m_pMesh->n_vertices() / 32 + 1), 1u, 1u);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT);
}

float OMT2D::ComputeError()
{
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_GradientBuffer);
    auto * pGradient = reinterpret_cast<float *>(glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY));
    float error = 0.0f;
    for (size_t i = 0; i < m_pMesh->n_vertices(); i++)
    {
        error += (pGradient[i] * pGradient[i]);
    }
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    return error;
}

uint32_t OMT2D::ComputeMissingPointNum()
{
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_CurrentAreaBuffer);
    auto * pCurrArea = reinterpret_cast<float *>(glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY));
    uint32_t num = 0;
    for (size_t i = 0; i < m_pMesh->n_vertices(); i++)
    {
        if (pCurrArea[i] == 0.0f) { num++; }
    }
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    return num;
}

void OMT2D::ComputeCellSite()
{
    m_CellPixelCountResetShader->UseProgram();
    m_CellPixelCountResetShader->SetInt("cellNum", static_cast<int>(m_pMesh->n_vertices()));
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_CellPixelCountBuffer);
    glDispatchCompute(static_cast<GLuint>(m_pMesh->n_vertices() / 32 + 1), 1u, 1u);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    m_ComputeCellSiteResetShader->UseProgram();
    m_ComputeCellSiteResetShader->SetInt("cellNum", static_cast<int>(m_pMesh->n_vertices()));
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_CellPositionAccumulateBuffer);
    glDispatchCompute(static_cast<GLuint>(m_pMesh->n_vertices() / 32 + 1), 1u, 1u);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    m_ComputeCellSiteShader->UseProgram();
    m_ComputeCellSiteShader->SetInt("powerDiagramMapWidth", m_RenderTargetWidth);
    m_ComputeCellSiteShader->SetInt("powerDiagramMapHeight", m_RenderTargetHeight);
    m_ComputeCellSiteShader->SetInt("powerDiagramMap", 0);
    glBindImageTexture(0, m_PowerDiagramTexture, 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, m_CellPixelCountBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_CellPositionAccumulateBuffer);
    glDispatchCompute(static_cast<GLuint>(m_RenderTargetWidth / 32 + 1), static_cast<GLuint>(m_RenderTargetHeight / 32 + 1), 1u);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    std::vector<int> cellCount(m_pMesh->n_vertices(), 0);
    std::vector<int> xAccu(m_pMesh->n_vertices(), 0);
    std::vector<int> yAccu(m_pMesh->n_vertices(), 0);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_CellPixelCountBuffer);
    auto * pCellPixelCount = reinterpret_cast<int *>(glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY));
    for (size_t i = 0; i < m_pMesh->n_vertices(); i++)
    {
        cellCount[i] = pCellPixelCount[i];
    }
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_CellPositionAccumulateBuffer);
    auto * pCellPositionAccu = reinterpret_cast<int *>(glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY));
    for (size_t i = 0; i < m_pMesh->n_vertices(); i++)
    {
        xAccu[i] = pCellPositionAccu[2 * i + 0];
        yAccu[i] = pCellPositionAccu[2 * i + 1];
    }
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    std::set<int> illedVertices;
    for (size_t i = 0; i < m_pMesh->n_vertices(); i++)
    {
        const auto vh = TriMesh::VertexHandle(static_cast<int>(i));
        float u = static_cast<float>(xAccu[i]) / static_cast<float>(cellCount[i]) / static_cast<float>(m_RenderTargetWidth);
        float v = static_cast<float>(yAccu[i]) / static_cast<float>(cellCount[i]) / static_cast<float>(m_RenderTargetHeight);
        if (cellCount[i] == 0)
        {
            u = -1.0f, v = -1.0f;
            illedVertices.insert(static_cast<int>(i));
        }
        m_pMesh->set_texcoord2D(vh, { u, v });
    }

    /* repair illed vertices which correspond to the missing point in power diagram */
    std::map<int, std::set<int>, std::greater<>> valenceVericesMap;
    std::map<int, int> vertexValenceMap;
    for (int illedVertex : illedVertices)
    {
        int valence = 0;
        const auto vh = TriMesh::VertexHandle(illedVertex);
        for (const auto vhvh : m_pMesh->vv_range(vh))
        {
            if (illedVertices.count(vhvh.idx()) == 0)
            {
                valence++;
            }
        }
        valenceVericesMap[valence].insert(vh.idx());
        vertexValenceMap[vh.idx()] = valence;
    }

    while (!illedVertices.empty())
    {
        auto & kv = *valenceVericesMap.begin();

        int valence = kv.first;
        int index = *kv.second.begin();

        TriMesh::TexCoord2D uv(0.0, 0.0);
        const auto vh = TriMesh::VertexHandle(index);
        for (const auto vhvh : m_pMesh->vv_range(vh))
        {
            if (illedVertices.count(vhvh.idx()) == 0)
            {
                const auto uvuv = m_pMesh->texcoord2D(vhvh);
                uv[0] += uvuv[0];
                uv[1] += uvuv[1];
            }
        }
        uv[0] /= static_cast<float>(valence);
        uv[1] /= static_cast<float>(valence);
        m_pMesh->set_texcoord2D(vh, uv);

        illedVertices.erase(index);
        vertexValenceMap.erase(index);
        kv.second.erase(index);
        if (kv.second.empty())
        {
            valenceVericesMap.erase(kv.first);
        }

        for (const auto vhvh : m_pMesh->vv_range(vh))
        {
            if (illedVertices.count(vhvh.idx()) != 0)
            {
                int & vhvhValence = vertexValenceMap[vhvh.idx()];
                std::set<int> & vhvhValenceVertexSet = valenceVericesMap[vhvhValence];
                vhvhValenceVertexSet.erase(vhvh.idx());
                if (vhvhValenceVertexSet.empty())
                {
                    valenceVericesMap.erase(vhvhValence);
                }
                vhvhValence++;
                valenceVericesMap[vhvhValence].insert(vhvh.idx());
            }
        }
    }

    /* optimize boundary vertex */
    for (const auto vh : m_pMesh->vertices())
    {
        if (!m_pMesh->is_boundary(vh)) { continue; }
        const TriMesh::TexCoord2D uv = m_pMesh->texcoord2D(vh);
        const TriMesh::TexCoord2D center(0.5, 0.5);
        const TriMesh::TexCoord2D vec = (uv - center).normalized();

        const std::vector<double> origin = { center[0], center[1] };
        const std::vector<double> dir = { vec[0], vec[1] };
        const std::vector<double> invdir = { 1.0 / dir[0], 1.0 / dir[1] };

        double tx, ty;

        if (invdir[0] > 0)
        {
            tx = (1.0 - origin[0]) * invdir[0];
        }
        else
        {
            tx = -origin[0] * invdir[0];
        }

        if (invdir[1] > 0)
        {
            ty = (1.0 - origin[1]) * invdir[1];
        }
        else
        {
            ty = -origin[1] * invdir[1];
        }

        m_pMesh->set_texcoord2D(vh, center + vec * std::min(tx, ty));
    }
}

void OMT2D::WriteHLBFGSErrorInfo(const std::string & filename)
{
    std::ofstream file(filename);
    for (const auto & pair : m_ErrorsHLBFGS)
    {
        file << pair.first << " " << pair.second << std::endl;
    }
    file.close();
    Logger::Info("Successfully write HLBFGS error info: \"%s\"!", filename.c_str());
}

void OMT2D::WriteGradientDescentErrorInfo(const std::string & filename)
{
    std::ofstream file(filename);
    for (const auto & pair : m_ErrorsGradientDescent)
    {
        file << pair.first << " " << pair.second << std::endl;
    }
    file.close();
    Logger::Info("Successfully write gradient descent error info: \"%s\"!", filename.c_str());
}

void OMT2D::WriteHLBFGSMissingPointInfo(const std::string & filename)
{
    std::ofstream file(filename);
    for (const auto & pair : m_MissingPointHLBFGS)
    {
        file << pair.first << " " << pair.second << std::endl;
    }
    file.close();
    Logger::Info("Successfully write HLBFGS missing point info: \"%s\"!", filename.c_str());
}

void OMT2D::WriteGradientDescentMissingPointInfo(const std::string & filename)
{
    std::ofstream file(filename);
    for (const auto & pair : m_MissingPointGradientDescent)
    {
        file << pair.first << " " << pair.second << std::endl;
    }
    file.close();
    Logger::Info("Successfully write gradient descent missing point info: \"%s\"!", filename.c_str());
}

void OMT2D::ShowAndSaveErrorFigure(const std::string & filename)
{
    std::vector<double> x1, y1;
    x1.reserve(m_ErrorsHLBFGS.size());
    y1.reserve(m_ErrorsHLBFGS.size());

    for (const auto & error : m_ErrorsHLBFGS)
    {
        x1.push_back(error.first / 1000.0);
        y1.push_back(error.second);
    }

    std::vector<double> x2, y2;
    x2.reserve(m_ErrorsGradientDescent.size() + 1);
    y2.reserve(m_ErrorsGradientDescent.size() + 1);

    if (!m_ErrorsHLBFGS.empty())
    {
        x2.push_back(x1.back());
        y2.push_back(y1.back());
    }

    for (const auto & error : m_ErrorsGradientDescent)
    {
        x2.push_back(error.first / 1000.0);
        y2.push_back(error.second);
    }

    matplotlibcpp::figure_size(480, 360);
    if (!m_ErrorsHLBFGS.empty())
    {
        matplotlibcpp::named_plot("LBFGS", x1, y1);
    }
    if (!m_ErrorsGradientDescent.empty())
    {
        matplotlibcpp::named_plot("Gradient Descent", x2, y2);
    }
    matplotlibcpp::title(R"(L2 Error $\left\|\nabla E(\mathbf{h})\right\|^2$)");
    matplotlibcpp::save(filename);
    matplotlibcpp::legend();
    matplotlibcpp::show();
    Logger::Info("Successfully write omt error figure: \"%s\"!", filename.c_str());
}

void OMT2D::ShowAndSaveMissingPointFigure(const std::string & filename)
{
    std::vector<double> x1, y1;
    x1.reserve(m_MissingPointHLBFGS.size());
    y1.reserve(m_MissingPointHLBFGS.size());

    for (const auto & error : m_MissingPointHLBFGS)
    {
        x1.push_back(error.first / 1000.0);
        y1.push_back(error.second);
    }

    std::vector<double> x2, y2;
    x2.reserve(m_MissingPointGradientDescent.size() + 2);
    y2.reserve(m_MissingPointGradientDescent.size() + 2);

    if (!m_MissingPointHLBFGS.empty())
    {
        x2.push_back(x1.back());
        y2.push_back(y1.back());
    }

    for (const auto & error : m_MissingPointGradientDescent)
    {
        x2.push_back(error.first / 1000.0);
        y2.push_back(error.second);
    }

    x2.push_back(x2.back() + 1);
    y2.push_back(0);

    matplotlibcpp::figure_size(480, 360);
    if (!m_MissingPointHLBFGS.empty())
    {
        matplotlibcpp::named_plot("LBFGS", x1, y1);
    }
    if (!m_MissingPointGradientDescent.empty())
    {
        matplotlibcpp::named_plot("Gradient Descent", x2, y2);
    }
    matplotlibcpp::title("Singular Point");
    matplotlibcpp::save(filename);
    matplotlibcpp::legend();
    matplotlibcpp::show();

    Logger::Info("Successfully write omt missing point figure: \"%s\"!", filename.c_str());
}
