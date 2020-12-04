CMAKE_MINIMUM_REQUIRED(VERSION 3.12)
PROJECT(SMI04)

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

ADD_DEFINITIONS(-D _USE_MATH_DEFINES)
ADD_DEFINITIONS(-D _CRT_SECURE_NO_WARNINGS)
ADD_DEFINITIONS(-D _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)
ADD_DEFINITIONS(-D NOMINMAX)

SET(PROJECT_INCLUDE_DIRS include)
SET(INCLUDE_DIRS
        ${PROJECT_INCLUDE_DIRS})

SET(SOURCE_FILES
        include/IDList.h
        include/IDSet.h
        include/PCBCGSolver.h
        include/Point2d.h
        include/Point3d.h
        include/PointTool.h
        include/PolarList.h
        include/Polyhedron.h
        include/SMI04Wrapper.h

        src/IDSet.cxx
        src/PCBCGSolver.cxx
        src/PointTool.cxx
        src/SMI04Wrapper.cpp)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_LIBRARY(${PROJECT_NAME} ${SOURCE_FILES})