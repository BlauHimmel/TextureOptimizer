CMAKE_MINIMUM_REQUIRED(VERSION 3.12)
PROJECT(EBP)

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

ADD_DEFINITIONS(-D _USE_MATH_DEFINES)
ADD_DEFINITIONS(-D _CRT_SECURE_NO_WARNINGS)
ADD_DEFINITIONS(-D _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)
ADD_DEFINITIONS(-D NOMINMAX)

ADD_SUBDIRECTORY(ext/triangle)

FIND_PACKAGE(OpenMesh REQUIRED NO_MODULE)
FIND_PACKAGE(Eigen3 3.3 REQUIRED NO_MODULE)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
FIND_PACKAGE(MKL REQUIRED)

SET(EIGEN3_LIBRARIES Eigen3::Eigen)
SET(OPENMESH_LIBRARIES OpenMeshCore OpenMeshTools)
SET(TRIANGLE_LIBRARIES triangle)
SET(EXT_LIBRARIES
        ${EIGEN3_LIBRARIES}
        ${OPENMESH_LIBRARIES}
        ${MKL_LIBRARIES}
        ${TRIANGLE_LIBRARIES})

SET(PROJECT_INCLUDE_DIRS include)
SET(EXT_INCLUDE_DIRS
        ext/triangle
        ${MKL_INCLUDE_DIR})
SET(INCLUDE_DIRS
        ${PROJECT_INCLUDE_DIRS}
        ${EXT_INCLUDE_DIRS})

SET(SOURCE_FILES
        include/BiljectivePara.h
        include/Common.h
        include/MeshDefinition.h
        include/Parafun.h
        include/PardisoSolver.h
        include/ShellData.h
        include/TriangleInterface.h
        include/EBPWrapper.h

        src/BiljectivePara.cpp
        src/Common.cpp
        src/MeshDefinition.cpp
        src/Parafun.cpp
        src/PardisoSolver.cpp
        src/ShellData.cpp
        src/TriangleInterface.cpp
        src/EBPWrapper.cpp)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_LIBRARY(${PROJECT_NAME} ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${EXT_LIBRARIES})
