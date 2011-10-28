# GTEST_* are defined by the find_package(GTest) call in the top-level
# CMakeLists.txt
include_directories(${GTEST_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/src)
add_executable(utest test/utest.cpp ${PROJECT_SOURCE_DIR}/src/rospack_cmdline.cpp ${PROJECT_SOURCE_DIR}/src/utils.cpp)
target_link_libraries(utest rospack ${GTEST_LIBRARIES})

# The prepare steps shouldn't really be tests, 
# but apparently tests can only depend on other tests.
add_test(NAME copy_test_files
         COMMAND cmake -E copy_directory ${PROJECT_SOURCE_DIR}/test ${CMAKE_CURRENT_BINARY_DIR})
add_test(NAME make_deep_directories
         COMMAND cmake -E chdir test/deep python deep.py)

add_test(NAME utestpy
         COMMAND python test/utest.py)
add_test(NAME uteststackpy
         COMMAND python test/utest_rosstack.py)
add_test(NAME utestcpp
         COMMAND utest)

set_tests_properties(make_deep_directories PROPERTIES DEPENDS copy_test_files)

set_tests_properties(utestpy PROPERTIES DEPENDS make_deep_directories)
