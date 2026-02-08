#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "vikit_ros::vikit_ros" for configuration "Release"
set_property(TARGET vikit_ros::vikit_ros APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vikit_ros::vikit_ros PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libvikit_ros.so"
  IMPORTED_SONAME_RELEASE "libvikit_ros.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS vikit_ros::vikit_ros )
list(APPEND _IMPORT_CHECK_FILES_FOR_vikit_ros::vikit_ros "${_IMPORT_PREFIX}/lib/libvikit_ros.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
