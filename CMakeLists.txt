cmake_minimum_required(VERSION 2.6.0)
set(CMAKE_BUILD_TYPE Release)
project(rospack)

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
message("Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message("Boost_LIBRARIES: ${Boost_LIBRARIES}")

include_directories(include ${PROJECT_SOURCE_DIR} ${Boost_INCLUDE_DIRS})
add_definitions(-DTIXML_USE_STL)
add_library(rospack SHARED src/rospack.cpp ${backcompat_source}
            tinyxml-2.5.3/tinyxml.cpp
            tinyxml-2.5.3/tinyxmlparser.cpp
            tinyxml-2.5.3/tinyxmlerror.cpp)
target_link_libraries(rospack ${Boost_LIBRARIES})
add_executable(rospackexe src/rospack_main.cpp)
set_target_properties(rospackexe PROPERTIES OUTPUT_NAME rospack)
target_link_libraries(rospackexe rospack ${Boost_LIBRARIES})
add_executable(rosstackexe src/rosstack_main.cpp)
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
  enable_testing()
  add_subdirectory(test)
endif()
