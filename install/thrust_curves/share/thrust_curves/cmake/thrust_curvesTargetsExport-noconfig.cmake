#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "thrust_curves::thrust_curves" for configuration ""
set_property(TARGET thrust_curves::thrust_curves APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(thrust_curves::thrust_curves PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libthrust_curves.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS thrust_curves::thrust_curves )
list(APPEND _IMPORT_CHECK_FILES_FOR_thrust_curves::thrust_curves "${_IMPORT_PREFIX}/lib/libthrust_curves.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
