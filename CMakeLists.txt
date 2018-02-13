cmake_minimum_required(VERSION 3.9)
project(ComplexityPositioning)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TOOL Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g  -lpthread")


# add cmake module
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# boost
find_package(Boost )

#OPENMP
find_package(OpenMP)
if (OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

#python dev
find_package(PythonLibs 3.5)

#Eigen
find_package(Eigen3 REQUIRED)

#BLAS speed up library.
find_package(BLAS)
find_package(LAPACK)


#Sophus
find_package(Sophus REQUIRED)
set(Sophus_LIBRARIES libSophus.so)


#GTSAM
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

# G2O
SET(G2O_ROOT /usr/local/include/g2o)
FIND_PACKAGE(G2O)
FIND_PACKAGE(CSparse)
FIND_PACKAGE(Cholmod)

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator
        g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
        g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
        g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper
        g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg
        g2o_types_data g2o_types_sim3 cxsparse
        g2o_types_slam3d_addons g2o_solver_cholmod cholmod)

######  include and library directories.
include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ./include
        /usr/local/cuda/include
        /home/steve/Cpp_Extent/Cpp_Extent
        ${PYTHON_INDCLUDE_PATH}
        /usr/include
        /usr/local/include
        ${Sophus_INCLUDE_DIRS}
        ${BLAS_INCLUDE_DIRS}
        /usr/local/include
        ${CSPARSE_INCLUDE_DIR}
        ${G2O_INCLUDE_DIR}
        ${Cholmod_INCLUDE_DIR}
        ${SuiteSparse_INCLUDE_DIR}
        ${GTSAM_INCLUDE_DIR}
        /home/steve/Code/AlgorithmWorkFlowTools/
        ${PYTHON_INCLUDE_DIRS}
        ./BayesStateEstimate/
        ${Boost_INCLUDE_DIRS}
)

link_directories(
        /usr/local/lib
        /usr/lib
        /usr/lib/x86_64-linux-gnu
        ${BLAS_LIBRARIES}
        ${LAPACK_LIBRARIES}
        ${PYTHON_LIBRARIES}
        ./include
)


add_executable(DualFeetLocalization GraphOptimization/src/DualFeetLocalization.cpp)

add_executable(KalmanFilterTester BayesStateEstimate/Test/KalmanFilterTester.cpp)
target_link_libraries(KalmanFilterTester ${PYTHON_LIBRARIES})

add_executable(KalmanFilterSimulatorTester BayesStateEstimate/Test/KalmanFilterSimulatorTester.cpp)
target_link_libraries(KalmanFilterSimulatorTester ${PYTHON_LIBRARIES})

add_executable(MonteCarloTester BayesStateEstimate/Test/MonteCarloTester.cpp)
target_link_libraries(MonteCarloTester ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})
