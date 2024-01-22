# Lines that begin with a # are comments
# set the minimum required version of cmake, usually the first line
cmake_minimum_required(VERSION 3.22)
# project_name sets the name of the project and causes cmake to
# find the c and c++ compilers
project(turtlelib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find your dependencies.
find_package(Doxygen)

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

# build just because Doxygen is missing
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/exercises/B6_frame_input.txt
    ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)

# Copy over the test_case_files directory to the build directory
execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_CURRENT_SOURCE_DIR}/test_case_files" "${CMAKE_CURRENT_BINARY_DIR}")

# Create a library.  Can specify if it is shared or static but usually
# you don't need or want to.
# name is the name of the library without the extension or lib prefix
# name creates a cmake "target"
add_library(geometry2d src/geometry2d.cpp)
add_library(se2d src/se2d.cpp)
add_library(svg src/svg.cpp)

# Use target_include_directories so that #include"mylibrary/header.hpp" works
# The use of the <BUILD_INTERFACE> and <INSTALL_INTERFACE> is because when
# Using the library from the build directory or after installation
# During build, the headers are read from the source code directory
# When used from the installed location, headers are in the
# system include/ directory
target_include_directories(geometry2d
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)
target_include_directories(se2d
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)
target_include_directories(svg
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

# specify additional compilation flags for the library
# Public causes the flags to propagate to anything
# that links against this library
target_compile_options(geometry2d PUBLIC -Wall -Wextra -pedantic)
target_compile_options(se2d PUBLIC -Wall -Wextra -pedantic)
target_compile_options(svg PUBLIC -Wall -Wextra -pedantic)

# Enable c++17 support.
# Public causes the features to propagate to anything
# that links against this library
target_compile_features(geometry2d PUBLIC cxx_std_17)
target_compile_features(se2d PUBLIC cxx_std_17)
target_compile_features(svg PUBLIC cxx_std_17)

# Create an executable from the following source code files
# The Name of the executable creates a cmake "target"
add_executable(frame_main src/frame_main.cpp)

# Use target_link_libraries to add dependencies to a "target"
# (e.g., a library or executable)
# This will automatically add all required library files
# that need to be linked
# and paths to th locations of header files
target_link_libraries(frame_main geometry2d se2d svg)

# install the include files by copying the whole include directory
install(DIRECTORY include/turtlelib DESTINATION include)

# Create a CMake Exported Target containing the lib and exe.
# Also create CMake Export called projet_name-targets
# The CMake Export contains files that allow other CMake projects
# to find this project. It must be installed separately.
install(TARGETS frame_main geometry2d se2d svg EXPORT turtlelib-targets)
# install(TARGETS turtlelib se2d EXPORT turtlelib-targets)

# The project_name-targets created by install(TARGETS) needs to be installed.
# install(EXPORT ...) will generate a file called project_name-config.cmake
# that contains the exported targets.
# After installation this file will then be found when calling
# find_package(project_name) from another cmake project
# A user can then target_link_libraries(target project_name::library)
# to use the libraries installed here
install(EXPORT turtlelib-targets
        FILE turtlelib-config.cmake
        NAMESPACE turtlelib::
        DESTINATION lib/cmake/${PROJECT_NAME})

include(CTest)
# CTest sets BUILD_TESTING to on. To disable tests add -DBUILD_TESTING=OFF when invoking cmake
if(BUILD_TESTING)
    # Find the Unit testing framework. In this example, Catch2
    find_package(Catch2 3 REQUIRED)

    # A test is just an executable that is linked against the unit testing library
    add_executable(test_geometry2D tests/test_geometry2d.cpp)
    target_link_libraries(test_geometry2D Catch2::Catch2WithMain geometry2d)

    # register the test with CTest, telling it what executable to run
    add_test(NAME geometry2D_test COMMAND test_geometry2D)

    add_executable(test_se2d tests/test_se2d.cpp)
    target_link_libraries(test_se2d Catch2::Catch2WithMain se2d geometry2d)
    add_test(NAME se2d_test COMMAND test_se2d)

    add_executable(test_svg tests/test_svg.cpp)
    target_link_libraries(test_svg Catch2::Catch2WithMain svg se2d geometry2d)
    add_test(NAME svg_test COMMAND test_svg)
endif()