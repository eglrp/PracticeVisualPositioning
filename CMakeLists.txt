cmake_minimum_required(VERSION 3.12)
project(PracticeVisualPositioning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TOOL Release)

#set(MKLROOT "/opt/intel/mkl/")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -fopenacc -lpthread -fopenmp -mavx -mfma ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g  -lpthread -fopenmp -mavx -mfma ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}-lopencv_line_descriptor -lopencv_optflow -lopencv_plot  -lopencv_stereo   -lopencv_xfeatures2d  -lopencv_calib3d -lopencv_features2d ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -L${MKLROOT}/lib/intel64 -Wl,--no-as-needed -lmkl_intel_lp64 -lmkl_gnu_thread -lmkl_core -lgomp -lpthread -lm -ldl ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -m64 -I${MKLROOT}/include ")

# CCACHE
find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
	set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
	set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif (CCACHE_FOUND)


find_package(Eigen3 REQUIRED)


#BLAS speed up library.
find_package(BLAS)
find_package(LAPACK)


#BOOST
find_package(Boost REQUIRED COMPONENTS filesystem system)


find_package(OpenCV REQUIRED)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

FIND_PACKAGE(CSparse)
if (CSPARSE_FOUND)
	message("founded csparse")
endif (CSPARSE_FOUND)


find_package(Ceres REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
					${Boost_INCLUDE_DIRS}
					${Eigen_INCLUDE_DIRS}
					./
					./include
					/usr/local/cuda/include
					/usr/include
					/usr/local/include
					${CSPARSE_INCLUDE_DIR}
					/home/steve/Code/googletest/googletest/include
					${CERES_INCLUDE_DIRS}
					/home/steve/Code/AlgorithmWorkFlowTools/
					/usr/include/python3.6m/
					)


link_directories(
		${Boost_LIBRARY_DIRS}
		/usr/local/lib
		/usr/lib
		/usr/lib/x86_64-linux-gnu
		${BLAS_LIBRARIES}
		${LAPACK_LIBRARIES}
)
set(AWF_LIBRARY /home/steve/Code/AlgorithmWorkFlowTools/lib/libAWF.so)
link_libraries(
		${AWF_LIBRARY}
)

aux_source_directory(
		./VisualOdometry SRC_FILES
)


aux_source_directory(./StereoVO StereoVO_FILES)

#aux_source_directory(
#		./CeresSolverTools
#		CERESTOOL
#)


add_executable(feature_track_server_test tester/feature_tracker_server_tester.cpp)
target_link_libraries(feature_track_server_test
					  ${OpenCV_LIBS}
					  ${Boost_LIBRARIES}
					  )

add_executable(huge_optimizer_tester tester/invalid/HugeOptimizerTester.cpp)
target_link_libraries(huge_optimizer_tester
					  ${OpenCV_LIBS}
					  ${Boost_LIBRARIES}
					  )

add_executable(slam_visual_server_test tester/slam_visual_server_tes.cpp)
target_link_libraries(
		slam_visual_server_test
		${OpenCV_LIBS}
		${Boost_LIBRARIES}
)

add_executable(mono_slam_tester tester/mono_slam_tester.cpp)
target_link_libraries(mono_slam_tester
					  ${OpenCV_LIBS}
					  ${Boost_LIBRARIES}
					  )

add_executable(StereoKittiTester tester/StereoKittiTester.cpp ${StereoVO_FILES})
target_link_libraries(
		StereoKittiTester
		${OpenCV_LIBS}
		${Boost_LIBRARIES}
		${CERES_LIBRARIES}

)


add_executable(SimDataTester tester/SimDataTester.cpp ${StereoVO_FILES})
target_link_libraries(
		SimDataTester
		${OpenCV_LIBS}
		${Boost_LIBRARIES}
		${CERES_LIBRARIES}
)

#add_executable(trajectory_tester tester/wtrajectory_test.cpp)
#target_link_libraries(trajectory_tester
#					  ${OpenCV_LIBS})
######################################################
# UNIT TESTING
######################################################
add_executable(
		runFeatureTrackServerTests
		Unit_Test/TEST_FeatureTrackServer.cpp
)

target_link_libraries(runFeatureTrackServerTests gtest gtest_main)

add_executable(runConfigServerTests
			   Unit_Test/TEST_ConfigServer.cpp)

target_link_libraries(runConfigServerTests
					  gtest gtest_main)

