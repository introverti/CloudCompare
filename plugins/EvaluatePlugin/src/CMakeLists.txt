
target_sources( ${PROJECT_NAME}
	PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/evaluateplugin.cpp
	${CMAKE_CURRENT_LIST_DIR}/evaluation.cpp
	${CMAKE_CURRENT_LIST_DIR}/evaluatedlg.cpp
	${CloudComparePlugins_SOURCE_DIR}/Innolog/innolog_base.cpp
	${CloudComparePlugins_SOURCE_DIR}/Innolog/innolog.cpp
	${CloudComparePlugins_SOURCE_DIR}/Innolog/rotate_log_file.cpp
)
