#
# files containing dictionary of useful projection
#

set(CONFIG_FILES
  proj.ini
)

set(PROJ_DICTIONARY
  world
  other.extra
  nad27
  GL27
  nad83
  nad.lst
  CH
  ITRF2000
  ITRF2008
  ITRF2014
)

#
# gridshift file
#

file(GLOB GSB_FILES  *.gsb)
file(GLOB GTX_FILES  *.gtx)
set(GRIDSHIFT_FILES ${GSB_FILES} ${GTX_FILES})

file(GLOB SCHEMA_FILES *.json)

set(ALL_SQL_IN "${CMAKE_CURRENT_BINARY_DIR}/all.sql.in")
set(PROJ_DB "${CMAKE_CURRENT_BINARY_DIR}/proj.db")
include(sql_filelist.cmake)

add_custom_command(
  OUTPUT ${PROJ_DB}
  COMMAND ${CMAKE_COMMAND} -E remove -f ${PROJ_DB}
  COMMAND ${CMAKE_COMMAND} "-DALL_SQL_IN=${ALL_SQL_IN}" "-DEXE_SQLITE3=${EXE_SQLITE3}" "-DPROJ_DB=${PROJ_DB}" "-DPROJ_VERSION=${PROJ_VERSION}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/generate_proj_db.cmake"
  COMMAND ${CMAKE_COMMAND} -E copy ${PROJ_DB} ${CMAKE_CURRENT_BINARY_DIR}/for_tests
  DEPENDS ${SQL_FILES} "${CMAKE_CURRENT_SOURCE_DIR}/generate_proj_db.cmake"
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  COMMENT "Generating proj.db"
  VERBATIM
)

add_custom_target(generate_proj_db ALL DEPENDS ${PROJ_DB})

if(NOT "${CMAKE_CURRENT_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_BINARY_DIR}")
  foreach(FILE ${CONFIG_FILES} ${PROJ_DICTIONARY} ${GRIDSHIFT_FILES})
    configure_file(${FILE} ${FILE} COPYONLY)
  endforeach()
endif()

# Copy select resource files in a for_tests subdirectory so that we are not
# influenced by the presence of other grids
# Note: this is done at configure/cmake time, not build time.
# So if you install new grids in the source data/ subdirectory, run cmake again
set(DATA_FOR_TESTS
    GL27
    nad27
    nad83
    ITRF2000)

execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/for_tests)
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/for_tests/tests)

foreach(FILE ${DATA_FOR_TESTS} ${CONFIG_FILES})
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${FILE} ${CMAKE_CURRENT_BINARY_DIR}/for_tests/${FILE} COPYONLY)
endforeach()

file(GLOB DATA_TESTS tests/*)
foreach(FILE ${DATA_TESTS})
    get_filename_component(FILENAME ${FILE} NAME)
    configure_file(${FILE} ${CMAKE_CURRENT_BINARY_DIR}/for_tests/tests/${FILENAME} COPYONLY)
endforeach()

set(DATA_FOR_TESTS_FROM_TESTS_SUBDIR
    alaska
    BETA2007.gsb
    conus
    MD
    ntf_r93.gsb
    ntv1_can.dat)
foreach(FILE ${DATA_FOR_TESTS_FROM_TESTS_SUBDIR})
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/tests/${FILE} ${CMAKE_CURRENT_BINARY_DIR}/for_tests/${FILE} COPYONLY)
endforeach()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/tests/egm96_15_downsampled.gtx ${CMAKE_CURRENT_BINARY_DIR}/for_tests/egm96_15.gtx COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/tests/ntv2_0_downsampled.gsb ${CMAKE_CURRENT_BINARY_DIR}/for_tests/ntv2_0.gsb COPYONLY)

#
#install
#
set(ALL_DATA_FILE
  ${CONFIG_FILES}
  ${PROJ_DICTIONARY}
  ${GRIDSHIFT_FILES}
  ${PROJ_DB}
  ${SCHEMA_FILES}
)
install(
  FILES ${ALL_DATA_FILE}
  DESTINATION ${PROJ_LIB_PATH}
)
