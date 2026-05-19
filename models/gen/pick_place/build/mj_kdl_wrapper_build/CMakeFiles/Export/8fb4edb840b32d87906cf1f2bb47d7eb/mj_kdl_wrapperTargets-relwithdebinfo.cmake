#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mj_kdl_wrapper::mj_kdl_wrapper" for configuration "RelWithDebInfo"
set_property(TARGET mj_kdl_wrapper::mj_kdl_wrapper APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(mj_kdl_wrapper::mj_kdl_wrapper PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libmj_kdl_wrapper.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libmj_kdl_wrapper.so"
  )

list(APPEND _cmake_import_check_targets mj_kdl_wrapper::mj_kdl_wrapper )
list(APPEND _cmake_import_check_files_for_mj_kdl_wrapper::mj_kdl_wrapper "${_IMPORT_PREFIX}/lib/libmj_kdl_wrapper.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
