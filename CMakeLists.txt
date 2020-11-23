CMAKE_MINIMUM_REQUIRED(VERSION 3.12)
PROJECT(TextureOptimizer)

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

# handle error when compiling IglDebugViewer.cpp in debug mode using msvc
# fatal error C1128: number of sections exceeded object file format limit: compile with /bigobj
IF (WIN32)
    IF (CMAKE_BUILD_TYPE MATCHES "Debug")
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
    ENDIF()
ENDIF()

ADD_DEFINITIONS(-D _USE_MATH_DEFINES)
ADD_DEFINITIONS(-D _CRT_SECURE_NO_WARNINGS)
ADD_DEFINITIONS(-D _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)
ADD_DEFINITIONS(-D NOMINMAX)
ADD_DEFINITIONS(-D CGAL_USE_BASIC_VIEWER)

FIND_PACKAGE(OpenMesh REQUIRED NO_MODULE)
FIND_PACKAGE(Eigen3 3.3 REQUIRED NO_MODULE)
FIND_PACKAGE(TBB REQUIRED tbb NO_MODULE)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakes)
FIND_PACKAGE(MKL REQUIRED)
FIND_PACKAGE(GLFW3 3.3 REQUIRED)
FIND_PACKAGE(CGAL REQUIRED)
find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)

ADD_SUBDIRECTORY(Libraries/SLIM)

SET(EIGEN3_LIBRARIES Eigen3::Eigen)
SET(OPENMESH_LIBRARIES OpenMeshCore OpenMeshTools)
SET(TBB_LIBRARIES ${TBB_IMPORTED_TARGETS})
SET(CGAL_LIBRARIES CGAL::CGAL CGAL::CGAL_Qt5)
SET(SLIM_LIBRARIES SLIM)
SET(EXT_LIBRARIES
        ${EIGEN3_LIBRARIES}
        ${OPENMESH_LIBRARIES}
        ${TBB_LIBRARIES}
        ${SLIM_LIBRARIES}
        ${MKL_LIBRARIES}
        ${GLFW3_LIBRARY}
        ${GLFW_LIBRARIES}
        ${CGAL_LIBRARIES})

SET(PROJECT_INCLUDE_DIRS Includes)
SET(EXT_INCLUDE_DIRS
        Libraries/Lyra/include
        Libraries/stb/include
        Libraries/progress_bar/include
        Libraries/libigl/include
        Libraries/SLIM/include
        Libraries/glad/include
        ${GLFW3_INCLUDE_DIR}
        ${MKL_INCLUDE_DIR})
SET(INCLUDE_DIRS
        ${PROJECT_INCLUDE_DIRS}
        ${EXT_INCLUDE_DIRS})

SET(EXT_SOURCE_FILES
        Libraries/glad/src/glad.c)
SET(SOURCE_FILES
        ${EXT_SOURCE_FILES}
        Sources/Texture.cpp
        Sources/TextureOptimizer.cpp
        Sources/Logger.cpp
        Sources/CutToDiskImpl.cpp
        Sources/BarycentricParameterization.cpp
        Sources/AtlasMerger.cpp
        Sources/MeshUtils.cpp
        Sources/IglDebugViewer.cpp
        Sources/OMT.cpp)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_EXECUTABLE(TextureOptimizer MainTextureOptimizer.cpp ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(TextureOptimizer ${EXT_LIBRARIES})

ADD_EXECUTABLE(TextureOptimizerV2 MainTextureOptimizerV2.cpp ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(TextureOptimizerV2 ${EXT_LIBRARIES})

ADD_EXECUTABLE(OMT MainOMT.cpp ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(OMT ${EXT_LIBRARIES})