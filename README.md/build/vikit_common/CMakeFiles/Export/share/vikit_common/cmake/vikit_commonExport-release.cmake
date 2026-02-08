#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "vikit_common::vikit_common" for configuration "Release"
set_property(TARGET vikit_common::vikit_common APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vikit_common::vikit_common PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libvikit_common.so"
  IMPORTED_SONAME_RELEASE "libvikit_common.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS vikit_common::vikit_common )
list(APPEND _IMPORT_CHECK_FILES_FOR_vikit_common::vikit_common "${_IMPORT_PREFIX}/lib/libvikit_common.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
