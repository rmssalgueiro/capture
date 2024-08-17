#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "static_trajectory_manager::static_trajectory_manager" for configuration ""
set_property(TARGET static_trajectory_manager::static_trajectory_manager APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(static_trajectory_manager::static_trajectory_manager PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libstatic_trajectory_manager.so"
  IMPORTED_SONAME_NOCONFIG "libstatic_trajectory_manager.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS static_trajectory_manager::static_trajectory_manager )
list(APPEND _IMPORT_CHECK_FILES_FOR_static_trajectory_manager::static_trajectory_manager "${_IMPORT_PREFIX}/lib/libstatic_trajectory_manager.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
