# CloudCompare example for standard plugins

# REPLACE ALL 'ExamplePlugin' OCCURENCES BY YOUR PLUGIN NAME
# AND ADAPT THE CODE BELOW TO YOUR OWN NEEDS!

# Add an option to CMake to control whether we build this plugin or not
option( PLUGIN_SINGLE_CALIBRATION "Install CalibPlugin" ON )

find_package(PCL 1.10 REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)
find_package(Eigen3)
find_package(yaml-cpp 0.5.2 REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core REQUIRED)

if ( PLUGIN_SINGLE_CALIBRATION )
	project( CalibPlugin )
	 
	AddPlugin( NAME ${PROJECT_NAME} )
	include_directories(
                    ${OpenCV_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${YAML_CPP_INCLUDE_DIR})
		
	add_subdirectory( include )
	add_subdirectory( src )
	
	# set dependencies to necessary libraries
	target_link_libraries( ${PROJECT_NAME} Eigen3::Eigen ${OpenCV_LIBS} ${PCL_LIBRARIES} yaml-cpp QPCL_PLUGIN_UTILS_LIB Qt5::Widgets)
endif()
