cmake_minimum_required(VERSION 2.6.0)
project(rospack)

find_package(catkin)


# These get set in the mingw cmake toolchain file, but are also oft used
# in other ros packages to identify as static or shared
if(ROS_BUILD_STATIC_LIBS OR BUILD_STATIC)
  message(STATUS "A global ROS_BUILD_STATIC or BUILD_STATIC found.")
  message(STATUS "Rospack building static objects.")
  add_definitions(-DROS_BUILD_STATIC_LIBS) # tinyxml.h:86
  set(ROSPACK_BUILD_STATIC TRUE CACHE BOOL "Build static libs (as opposed to shared libs)")
else()
  message(STATUS "Rospack building shared objects.")
  set(ROSPACK_BUILD_STATIC FALSE CACHE BOOL "Build static libs (as opposed to shared libs)")
endif()

add_definitions(-Wall)

set(API_BACKCOMPAT_V1 "YES"
    CACHE BOOL "Whether to enable backwards compatibility with old C++ API")
if(API_BACKCOMPAT_V1)
  add_definitions(-DROSPACK_API_BACKCOMPAT_V1)
  set(backcompat_source src/rospack_backcompat.cpp)
endif()

find_package(Boost 1.40.0 COMPONENTS system filesystem program_options)
if(NOT Boost_FOUND)
  message(FATAL_ERROR "Couldn't find Boost")
endif()

include_directories(include ${PROJECT_SOURCE_DIR} ${Boost_INCLUDE_DIRS})
add_definitions(-DTIXML_USE_STL)

if(ROSPACK_BUILD_STATIC)
  add_library(rospack STATIC src/rospack.cpp ${backcompat_source}
            src/utils.cpp
            tinyxml-2.5.3/tinyxml.cpp
            tinyxml-2.5.3/tinyxmlparser.cpp
            tinyxml-2.5.3/tinyxmlerror.cpp)
else()
  add_library(rospack SHARED src/rospack.cpp ${backcompat_source}
            src/utils.cpp
            tinyxml-2.5.3/tinyxml.cpp
            tinyxml-2.5.3/tinyxmlparser.cpp
            tinyxml-2.5.3/tinyxmlerror.cpp)
endif()
target_link_libraries(rospack ${Boost_LIBRARIES})
add_executable(rospackexe src/rospack_main.cpp src/rospack_cmdline.cpp)
set_target_properties(rospackexe PROPERTIES OUTPUT_NAME rospack)
target_link_libraries(rospackexe rospack ${Boost_LIBRARIES})
add_executable(rosstackexe src/rosstack_main.cpp src/rospack_cmdline.cpp)
target_link_libraries(rosstackexe rospack ${Boost_LIBRARIES})
set_target_properties(rosstackexe PROPERTIES OUTPUT_NAME rosstack)

install(TARGETS rospack rospackexe rosstackexe
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY include/rospack
        DESTINATION include)
# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

find_package(GTest)
if(GTEST_FOUND)
  message("Found gtest; building C++ unit tests")
  enable_testing()
  add_subdirectory(test)
endif()

find_program(DOXYGEN doxygen)
if(DOXYGEN)
  message("Found doxygen; 'make docs' to build documentation")
  add_custom_target(docs
                    COMMAND doxygen Doxyfile
                    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endif()

install(DIRECTORY include
  DESTINATION include)

catkin_project(rospack
  VERSION 1.0.0
  LIBRARIES rospack
  INCLUDE_DIRS include
  )

