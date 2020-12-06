#include "OMT.hpp"

#include <memory>
#include <random>

#include "Logger.hpp"

/*******************************************************************************************************/

const char * pDrawPowerDiagramShaderVS = R"(
#version 430 core
layout(location = 0) in vec4 plane;
layout(location = 1) in vec3 color;
layout(location = 2) in float planeIndex;

out vec4 vColor;

void main()
{
    gl_Position = vec4(plane.xyz, 1.0);
    vColor = vec4(color, planeIndex);
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
        grad[x] = targArea[x] - currArea[x];
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

uniform int cellNum;
uniform float step;

void main()
{
    int x = int(gl_GlobalInvocationID.x);
    if (x < cellNum)
	{
        p[x].z += (grad[x] * step);
    }
}
)";

/*******************************************************************************************************/

bool OMT2D::Initialize()
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

    glfwSetWindowAttrib(m_pWindow, GLFW_RESIZABLE, GLFW_FALSE);
    glfwSetWindowAttrib(m_pWindow, GLFW_MAXIMIZED , GLFW_FALSE);

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
    InitTargetArea(m_pMesh, m_TargetAreas);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_TargetAreaBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, m_TargetAreas.size() * sizeof(float), m_TargetAreas.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    if (m_PlaneBuffer != GLuint(-1)) { glDeleteBuffers(1, &m_PlaneBuffer); }
    glGenBuffers(1, &m_PlaneBuffer);
    InitPlaneEquation(m_pMesh, m_Planes);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_PlaneBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, m_Planes.size() * sizeof(Plane), m_Planes.data(), GL_DYNAMIC_DRAW);
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

    if (m_PlaneIndexBuffer != GLuint(-1)) { glDeleteBuffers(1, &m_PlaneIndexBuffer); }
    glGenBuffers(1, &m_PlaneIndexBuffer);
    InitPlaneIndex(m_pMesh, m_PlaneIndices);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_PlaneIndexBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, m_PlaneIndices.size() * sizeof(float), m_PlaneIndices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

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
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        Logger::Error("ERROR::FRAMEBUFFER:: Framebuffer is not complete!");
        return false;
    }

    if (m_PowerDiagramVAO != GLuint(-1)) { glDeleteVertexArrays(1, &m_PowerDiagramVAO); }
    glGenVertexArrays(1, &m_PowerDiagramVAO);
    glBindVertexArray(m_PowerDiagramVAO);

    glBindBuffer(GL_ARRAY_BUFFER, m_PlaneBuffer);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Plane), nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, m_CellColorBuffer);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Color), nullptr);

    glBindBuffer(GL_ARRAY_BUFFER, m_PlaneIndexBuffer);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(float), nullptr);

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

    m_UpdatePlaneShader = std::make_unique<Shader>("Update Plane Shader");
    m_UpdatePlaneShader->CompileShaderFromString(pUpdatePlaneShaderCS, ESHADER_TYPE_COMPUTE);
    m_UpdatePlaneShader->LinkProgram();

    /*******************************************************************************************************/

    return true;
}

void OMT2D::Run(float step)
{
    while (!glfwWindowShouldClose(m_pWindow))
    {
        DrawPowerDiagram();
        ComputeCellStats();
        UpdatePlane(step);
        glfwSwapBuffers(m_pWindow);
        glfwPollEvents();
    }

    glfwTerminate();
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
        size_t Index = 0;

        for (const auto & vh : pMesh->fv_range(fh))
        {
            points[Index++] = pMesh->point(vh);
        }

        const auto & va = points[0];
        const auto & vb = points[1];
        const auto & vc = points[2];

        auto lab = va - vb;
        auto lac = vc - vb;

        Real area = lab.cross(lac).norm() * 0.5;
        totalMeshArea += area;
    }

    for (const auto & vh : pMesh->vertices())
    {
        Real oneRingMeshArea = 0.0;
        for (const auto & fh : pMesh->vf_range(vh))
        {
            TriMesh::Point points[3];
            size_t Index = 0;

            for (const auto & fhvh : pMesh->fv_range(fh))
            {
                points[Index++] = pMesh->point(fhvh);
            }

            const auto & va = points[0];
            const auto & vb = points[1];
            const auto & vc = points[2];

            auto lab = va - vb;
            auto lac = vc - vb;

            Real area = lab.cross(lac).norm() * 0.5;
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
        planes[vh.idx()].padding = 0.0f;
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

void OMT2D::InitPlaneIndex(const TriMesh * pMesh, std::vector<float> & indices)
{
    indices.resize(pMesh->n_vertices(), 0);

    int idx = 0;
    for (const auto & vh : pMesh->vertices())
    {
        indices[idx++] = static_cast<float>(vh.idx());
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

void OMT2D::DrawPowerDiagram()
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

void OMT2D::UpdatePlane(float step)
{
    m_UpdatePlaneShader->UseProgram();
    m_UpdatePlaneShader->SetInt("cellNum", static_cast<int>(m_pMesh->n_vertices()));
    m_UpdatePlaneShader->SetFloat("step", step);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_PlaneBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, m_GradientBuffer);
    glDispatchCompute(static_cast<GLuint>(m_pMesh->n_vertices() / 32 + 1), 1u, 1u);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT);
}


