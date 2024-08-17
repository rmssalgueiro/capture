#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "autopilot_controllers::autopilot_controllers" for configuration ""
set_property(TARGET autopilot_controllers::autopilot_controllers APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(autopilot_controllers::autopilot_controllers PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libautopilot_controllers.so"
  IMPORTED_SONAME_NOCONFIG "libautopilot_controllers.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS autopilot_controllers::autopilot_controllers )
list(APPEND _IMPORT_CHECK_FILES_FOR_autopilot_controllers::autopilot_controllers "${_IMPORT_PREFIX}/lib/libautopilot_controllers.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
