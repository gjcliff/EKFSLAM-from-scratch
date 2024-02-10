#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "nuturtle_control::nuturtle_control__rosidl_generator_py" for configuration "Debug"
set_property(TARGET nuturtle_control::nuturtle_control__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(nuturtle_control::nuturtle_control__rosidl_generator_py PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libnuturtle_control__rosidl_generator_py.so"
  IMPORTED_SONAME_DEBUG "libnuturtle_control__rosidl_generator_py.so"
  )

list(APPEND _cmake_import_check_targets nuturtle_control::nuturtle_control__rosidl_generator_py )
list(APPEND _cmake_import_check_files_for_nuturtle_control::nuturtle_control__rosidl_generator_py "${_IMPORT_PREFIX}/lib/libnuturtle_control__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)