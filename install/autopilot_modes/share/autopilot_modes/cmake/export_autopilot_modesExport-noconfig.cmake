#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "autopilot_modes::autopilot_modes" for configuration ""
set_property(TARGET autopilot_modes::autopilot_modes APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(autopilot_modes::autopilot_modes PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libautopilot_modes.so"
  IMPORTED_SONAME_NOCONFIG "libautopilot_modes.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS autopilot_modes::autopilot_modes )
list(APPEND _IMPORT_CHECK_FILES_FOR_autopilot_modes::autopilot_modes "${_IMPORT_PREFIX}/lib/libautopilot_modes.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
