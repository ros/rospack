cmake_minimum_required(VERSION 2.8.3)
project(rospack)
find_package(catkin REQUIRED)
catkin_package(
  LIBRARIES rospack
  INCLUDE_DIRS include
)

# BUILD_SHARED_LIBS is a global cmake variable (usually defaults to on) that determines
# the build type of libraries:
#   http://www.cmake.org/cmake/help/cmake-2-8-docs.html#variable:BUILD_SHARED_LIBS
if(BUILD_SHARED_LIBS)
  message(STATUS "Rospack building shared objects.")
else()
  message(STATUS "Rospack building static objects.")
  add_definitions(-DROS_BUILD_STATIC_LIBS) # tinyxml.h:86
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

add_library(rospack src/rospack.cpp ${backcompat_source}
            src/utils.cpp src/rospack_cmdline.cpp
            tinyxml-2.5.3/tinyxml.cpp
            tinyxml-2.5.3/tinyxmlparser.cpp
            tinyxml-2.5.3/tinyxmlerror.cpp)

target_link_libraries(rospack ${Boost_LIBRARIES})
add_executable(rospackexe src/rospack_main.cpp)
# Set the name, and make it a "global" executable
set_target_properties(rospackexe
                      PROPERTIES OUTPUT_NAME rospack
                                 RUNTIME_OUTPUT_DIRECTORY ${CATKIN_BUILD_PREFIX}/bin)
target_link_libraries(rospackexe rospack ${Boost_LIBRARIES})
add_executable(rosstackexe src/rosstack_main.cpp)
target_link_libraries(rosstackexe rospack ${Boost_LIBRARIES})
# Set the name, and make it a "global" executable
set_target_properties(rosstackexe
                      PROPERTIES OUTPUT_NAME rosstack
                                 RUNTIME_OUTPUT_DIRECTORY ${CATKIN_BUILD_PREFIX}/bin)

install(TARGETS rospack rospackexe rosstackexe
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

if(GTEST_FOUND)
  add_subdirectory(test)
endif()

if(DOXYGEN_FOUND)
  add_custom_target(rospack-docs
                    COMMAND doxygen Doxyfile
                    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endif()
