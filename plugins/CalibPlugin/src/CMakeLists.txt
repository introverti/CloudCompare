
target_sources( ${PROJECT_NAME}
	PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/calibplugin.cpp
	${CMAKE_CURRENT_LIST_DIR}/calibration.cpp
	${CMAKE_CURRENT_LIST_DIR}/calibdlg.cpp
	${CloudComparePlugins_SOURCE_DIR}/Innolog/innolog_base.cpp
	${CloudComparePlugins_SOURCE_DIR}/Innolog/innolog.cpp
	${CloudComparePlugins_SOURCE_DIR}/Innolog/rotate_log_file.cpp
)