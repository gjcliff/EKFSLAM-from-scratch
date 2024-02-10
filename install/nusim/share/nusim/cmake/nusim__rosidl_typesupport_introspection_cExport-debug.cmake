#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "nusim::nusim__rosidl_typesupport_introspection_c" for configuration "Debug"
set_property(TARGET nusim::nusim__rosidl_typesupport_introspection_c APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(nusim::nusim__rosidl_typesupport_introspection_c PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libnusim__rosidl_typesupport_introspection_c.so"
  IMPORTED_SONAME_DEBUG "libnusim__rosidl_typesupport_introspection_c.so"
  )

list(APPEND _cmake_import_check_targets nusim::nusim__rosidl_typesupport_introspection_c )
list(APPEND _cmake_import_check_files_for_nusim::nusim__rosidl_typesupport_introspection_c "${_IMPORT_PREFIX}/lib/libnusim__rosidl_typesupport_introspection_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
