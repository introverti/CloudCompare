
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/calibplugin.h
		${CMAKE_CURRENT_LIST_DIR}/calibration.h
		${CMAKE_CURRENT_LIST_DIR}/calibdlg.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
