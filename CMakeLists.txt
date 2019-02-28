cmake_minimum_required(VERSION 3.13)
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

#GTSAM
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)


#BOOST
find_package(Boost REQUIRED COMPONENTS filesystem system)


find_package(OpenCV REQUIRED)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# G2O
# G2O
SET(G2O_ROOT /usr/local/include/g2o)
FIND_PACKAGE(G2O)
FIND_PACKAGE(CSparse)
if (CSPARSE_FOUND)
	message("founded csparse")
endif (CSPARSE_FOUND)

#INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})
SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator
	g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
	g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
	g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper
	g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg
	g2o_types_data g2o_types_sim3 cxsparse
	g2o_types_slam3d_addons)


include_directories(${OpenCV_INCLUDE_DIRS}
					${GTSAM_INCLUDE_DIR}
					${Boost_INCLUDE_DIRS}
					${Eigen_INCLUDE_DIRS}
					./
					./include
					/usr/local/cuda/include
					/usr/include
					/usr/local/include
					${CSPARSE_INCLUDE_DIR}
					${G2O_INCLUDE_DIR}
					/home/steve/Code/googletest/googletest/include
					)


#add_executable( DisplayImage DisplayImage.cpp )
#target_link_libraries( DisplayImage ${OpenCV_LIBS} )
link_directories(
		#		/opt/intel/mkl/lib/intel64/
		#		/opt/intel/mkl/lib/intel64_lin/
		${Boost_LIBRARY_DIRS}
		/usr/local/lib
		/usr/lib
		/usr/lib/x86_64-linux-gnu
		${BLAS_LIBRARIES}
		${LAPACK_LIBRARIES}


)
#target_link_libraries(
#
#)

aux_source_directory(
		./VisualOdometry SRC_FILES
)



add_executable(feature_track_server_test tester/feature_tracker_server_tester.cpp)
target_link_libraries(feature_track_server_test
					  ${OpenCV_LIBS}
					  ${G2O_LIBS} ${CSPARSE_LIBRARY}
					  ${Boost_LIBRARIES}
					  )



add_executable(slam_visual_server_test tester/slam_visual_server_tes.cpp)
target_compile_definitions(
		slam_visual_server_test
		${OpenCV_LIBS}
		${Boost_LIBRARIES}
)
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


