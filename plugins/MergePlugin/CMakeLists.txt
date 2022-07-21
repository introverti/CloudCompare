# CloudCompare example for standard plugins

# REPLACE ALL 'ExamplePlugin' OCCURENCES BY YOUR PLUGIN NAME
# AND ADAPT THE CODE BELOW TO YOUR OWN NEEDS!

# Add an option to CMake to control whether we build this plugin or not
option( PLUGIN_MERGE_PC "Install MergePlugin" ON )

if ( PLUGIN_MERGE_PC )
	project( MergePlugin )
	 
	AddPlugin( NAME ${PROJECT_NAME} )
		
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )
endif()
