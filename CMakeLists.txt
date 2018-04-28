cmake_minimum_required(VERSION 3.5)
project(ComplexityPositioning)#LANGUAGES CXX CUDA)


set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TOOL Release)
set(MKLROOT "/opt/intel/mkl/")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -fopenacc -lpthread -fopenmp -mavx -mfma ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -lpthread -fopenmp -mavx -mfma ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g  -lpthread -fopenmp -mavx -mfma ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -L${MKLROOT}/lib/intel64 -Wl,--no-as-needed -lmkl_intel_lp64 -lmkl_gnu_thread -lmkl_core -lgomp -lpthread -lm -ldl ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -m64 -I${MKLROOT}/include ")

# Configure CCache if available
find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
	set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
	set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif (CCACHE_FOUND)

# add cmake module
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# packages
#find_package(CUDA REQUIRED)
#find_package(Thrust REQUIRED)
# nvcc flags
#set(CUDA_NVCC_FLAGS -gencode arch=compute_30,code=sm_30;-G;-g)

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
set(Sophus_LIBRARIES)


#GTSAM
#find_package(GTSAM REQUIRED) # Uses installed package
#include_directories(${GTSAM_INCLUDE_DIR})
#set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

#find_package(GTSAMCMakeTools)
#include(GtsamMakeConfigFile)
#include(GtsamBuildTypes)
#include(GtsamTesting)

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
		#		${GTSAM_INCLUDE_DIR}
		/home/steve/Code/AlgorithmWorkFlowTools/
		${PYTHON_INCLUDE_DIRS}
		${Boost_INCLUDE_DIRS}
		${BoostCompute_INCLUDE_DIRS}
		${OpenCL_LIBRARY}
		/opt/intel/mkl/include


		BayesStateEstimate/
		GraphOptimization/
		GpuAlgorithm/
)

link_directories(
		/usr/local/lib
		/usr/lib
		/usr/lib/x86_64-linux-gnu
		${BLAS_LIBRARIES}
		${LAPACK_LIBRARIES}
		${PYTHON_LIBRARIES}
		./include
		${OpenCL_LIBRARY}
		/opt/intel/mkl/lib/intel64/
		${Sophus_LIBRARIES}
		#        /opt/intel/mkl/lib/intel64_lin/
)

#link_libraries(
#        /home/steve/Code/AlgorithmWorkFlowTools/lib/libAWF.so
#)
set(AWF_LIBRARY /home/steve/Code/AlgorithmWorkFlowTools/lib/libAWF.so)
link_libraries(
		${AWF_LIBRARY}

		${Sophus_LIBRARIES}
)

#add_library(BayesStateEstimate  STATIC BayesStateEstimate/BSE.h)

#add_subdirectory(BayesStateEstimate)
#add_subdirectory(GraphOptimization)


add_executable(DualFeetLocalization GraphOptimization/src/DualFeetLocalization.cpp)
target_link_libraries(DualFeetLocalization ${PYTHON_LIBRARIES}
					  ${Sophus_LIBRARIES}
					  ${G2O_LIBS}
					  ${CSPARSE_LIBRARY}
					  ${PYTHON_LIBRARIES}
					  ${SuiteSparse_LIBRARIES})

add_executable(DualFeetIndividually GraphOptimization/src/DualFeetIndividually.cpp BayesStateEstimate/BayesFilter/KalmanFilter/KFComplexOpenShoe.h)
target_link_libraries(DualFeetIndividually ${PYTHON_LIBRARIES}
					  ${Sophus_LIBRARIES} ${G2O_LIBS} ${CSPARSE_LIBRARY} ${PYTHON_LIBRARIES} ${SuiteSparse_LIBRARIES})

add_executable(ImuUwbRobustFusion GraphOptimization/src/ImuUwbRobustFusionGraph.cpp)
target_link_libraries(ImuUwbRobustFusion ${PYTHON_LIBRARIES}
					  ${Sophus_LIBRARIES}
					  ${G2O_LIBS}
					  ${CSPARSE_LIBRARY}
					  ${PYTHON_LIBRARIES}
					  ${SuiteSparse_LIBRARIES})


add_executable(KalmanFilterTester BayesStateEstimate/Test/KalmanFilterTester.cpp)
target_link_libraries(KalmanFilterTester ${PYTHON_LIBRARIES})

add_executable(KFComplexityTester BayesStateEstimate/Test/KFComplexityTest.cpp)
target_link_libraries(KFComplexityTester ${PYTHON_LIBRARIES})

add_executable(KFCImuTester BayesStateEstimate/Test/KFComplexityImuOnlyTest.cpp BayesStateEstimate/BayesFilter/KalmanFilter/UKFComplexCraft.h)
target_link_libraries(KFCImuTester ${PYTHON_LIBRARIES})

add_executable(RotationTester BayesStateEstimate/Test/RotationReliableTester.cpp)
target_link_libraries(RotationTester ${PYTHON_LIBRARIES})

add_executable(XsenseIntegrateTest BayesStateEstimate/Test/XsenseIntegrateTest.cpp)
target_link_libraries(XsenseIntegrateTest ${PYTHON_LIBRARIES})

add_executable(BJFusionTest BayesStateEstimate/Test/BJFusionTest.cpp)
target_link_libraries(BJFusionTest ${PYTHON_LIBRARIES})


add_executable(UwbOptTest BayesStateEstimate/Test/UwbOptimizationTest.cpp)
target_link_libraries(UwbOptTest ${PYTHON_LIBRARIES})


add_executable(KalmanFilterSimulatorTester BayesStateEstimate/Test/KalmanFilterSimulatorTester.cpp)
target_link_libraries(KalmanFilterSimulatorTester ${PYTHON_LIBRARIES})

add_executable(MonteCarloTester BayesStateEstimate/Test/MonteCarloTester.cpp)
target_link_libraries(MonteCarloTester ${PYTHON_LIBRARIES} ${Boost_LIBRARIES} ${OpenCL_LIBRARY})

#cuda_add_executable(MonteCarloThrustTester
#        BayesStateEstimate/Test/MonteCarloTesterCuda.cpp)
#target_link_libraries(MonteCarloThrustTester )

#add_executable(GpuPF BayesStateEstimate/Test/GpuParticleFilter.cpp)
#target_link_libraries(GpuPF ${PYTHON_LIBRARIES} ${Boost_LIBRARIES} ${OpenCL_LIBRARY})

#CUDA_ADD_EXECUTABLE(test_cu GpuAlgorithm/src/test_cu.cu)
#add_executable(test_cu GpuAlgorithm/src/test_cu.cu)
#set_property(TARGET test_cu PROPERTY CUDA_SEPARABLE_COMPILATION ON)

#CUDA_ADD_EXECUTABLE(CUDA_Test ${CPP_LIST} ${CU_LIST})
