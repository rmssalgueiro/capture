#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "static_trajectories::static_trajectories" for configuration ""
set_property(TARGET static_trajectories::static_trajectories APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(static_trajectories::static_trajectories PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libstatic_trajectories.so"
  IMPORTED_SONAME_NOCONFIG "libstatic_trajectories.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS static_trajectories::static_trajectories )
list(APPEND _IMPORT_CHECK_FILES_FOR_static_trajectories::static_trajectories "${_IMPORT_PREFIX}/lib/libstatic_trajectories.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
