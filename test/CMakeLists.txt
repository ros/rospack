#if(ROSBUILD)
#  # this is for rosbuild2 integration, you can ignore this
#  include(rosbuild.cmake OPTIONAL)
#  return()
#endif()
#cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#rosbuild_init()
#rosbuild_add_boost_directories()
#rosbuild_add_pyunit(test/utest.py TIMEOUT 180.0)
#rosbuild_add_pyunit(test/utest_rosstack.py TIMEOUT 180.0)
#
#rosbuild_add_gtest(test/utest test/utest.cpp TIMEOUT 180.0)
#rosbuild_link_boost(test/utest thread)

# GTEST_* are defined by the find_package(GTest) call in the top-level
# CMakeLists.txt
include_directories(${GTEST_INCLUDE_DIRS})
add_executable(utest test/utest.cpp)
target_link_libraries(utest rospack ${GTEST_LIBRARIES})

# The prepare steps shouldn't really be tests, 
# but apparently tests can only depend on other tests.
add_test(NAME copy_test_files
         COMMAND cmake -E copy_directory ${PROJECT_SOURCE_DIR}/test ${CMAKE_CURRENT_BINARY_DIR})
add_test(NAME make_deep_directories
         COMMAND cmake -E chdir test/deep python deep.py)

add_test(NAME utestpy
         COMMAND python test/utest.py)
add_test(NAME utestcpp
         COMMAND utest)

set_tests_properties(make_deep_directories PROPERTIES DEPENDS copy_test_files)

set_tests_properties(utestpy PROPERTIES DEPENDS make_deep_directories)
