CMAKE_MINIMUM_REQUIRED(VERSION 3.12)

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

ADD_DEFINITIONS(-D _USE_MATH_DEFINES)
ADD_DEFINITIONS(-D _CRT_SECURE_NO_WARNINGS)
ADD_DEFINITIONS(-D _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)

FIND_PACKAGE(Eigen3 3.3 REQUIRED NO_MODULE)
FIND_PACKAGE(TBB REQUIRED tbb NO_MODULE)

SET(SLIM_EXT_LIBRARIES Eigen3::Eigen ${TBB_IMPORTED_TARGETS})

SET(SLIM_INCLUDE_DIRS include ../libigl/include)
SET(SLIM_SOURCE_FILES
        include/AlgorithmStage.h
        include/eigen_stl_utils.h
        include/Energy.h
        include/FastLsBuildUtils.h
        include/GlobalLocalParametrization.h
        include/LinesearchParametrizer.h
        include/LocalWeightedArapParametrizer.h
        include/Param_State.h
        include/parametrization_utils.h
        include/ParametrizationAlgorithm.h
        include/StateManager.h
        include/SymmetricDirichlet.h
        include/SLIMWrapper.hpp

        src/eigen_stl_utils.cpp
        src/FastLsBuildUtils.cpp
        src/GlobalLocalParametrization.cpp
        src/LinesearchParametrizer.cpp
        src/LocalWeightedArapParametrizer.cpp
        src/Param_State.cpp
        src/parametrization_utils.cpp
        src/StateManager.cpp
        src/SymmetricDirichlet.cpp
        src/SLIMWrapper.cpp)

INCLUDE_DIRECTORIES(${SLIM_INCLUDE_DIRS})
ADD_LIBRARY(SLIM ${SLIM_SOURCE_FILES})
TARGET_LINK_LIBRARIES(SLIM ${SLIM_EXT_LIBRARIES})