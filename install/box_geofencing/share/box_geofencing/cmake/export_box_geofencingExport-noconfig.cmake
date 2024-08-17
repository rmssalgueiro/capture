#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "box_geofencing::box_geofencing" for configuration ""
set_property(TARGET box_geofencing::box_geofencing APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(box_geofencing::box_geofencing PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libbox_geofencing.so"
  IMPORTED_SONAME_NOCONFIG "libbox_geofencing.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS box_geofencing::box_geofencing )
list(APPEND _IMPORT_CHECK_FILES_FOR_box_geofencing::box_geofencing "${_IMPORT_PREFIX}/lib/libbox_geofencing.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
