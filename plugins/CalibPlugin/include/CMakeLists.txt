
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/calibplugin.h
		${CMAKE_CURRENT_LIST_DIR}/calibration.h
		${CMAKE_CURRENT_LIST_DIR}/calibdlg.h
		${CloudComparePlugins_SOURCE_DIR}/Innolog/innolog_base.h
		${CloudComparePlugins_SOURCE_DIR}/Innolog/innolog.h
		${CloudComparePlugins_SOURCE_DIR}/Innolog/rotate_log_file.h
		${CloudComparePlugins_SOURCE_DIR}/Innolog/logger.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
