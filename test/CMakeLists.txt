# We replicate some macros from rosunit-extras.cmake here, to avoid
# depending on rosunit.

# This is a macro to ensure that ${PROJECT_NAME}_CACHE gets set in a higher
# scope where we can check it later.
macro(rospack_append_test_to_cache CACHENAME)
  set(cachefile ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}.${CACHENAME})
  if(NOT ${PROJECT_NAME}_CACHE)
    file(WRITE ${cachefile} "#\n# This is ${cachefile}\n#\n")
    # Write out the directory that we'll use as CWD later when running
    # tests
    file(APPEND ${cachefile} "${PROJECT_SOURCE_DIR}\n")
    set(${PROJECT_NAME}_CACHE TRUE)
    set(${PROJECT_NAME}_CACHE TRUE PARENT_SCOPE)
    # One test target per project
    add_custom_target(${PROJECT_NAME}_run_tests
      COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE}
      ${catkin_EXTRAS_DIR}/test/runtests.py ${cachefile})
    add_dependencies(test ${PROJECT_NAME}_run_tests)
    add_dependencies(${PROJECT_NAME}_run_tests tests)
  endif()

  # gotcha:  you need a newline or the message doesn't appear
  # in the cache.   pfft.
  file(APPEND ${cachefile} "${ARGN}\n")
endmacro()

function(rospack_add_pyunit file)
  # Look for optional TIMEOUT argument, #2645
  parse_arguments(_pyunit "TIMEOUT;WORKING_DIRECTORY" "" ${ARGN})
  if(NOT _pyunit_TIMEOUT)
    set(_pyunit_TIMEOUT 60.0)
  endif()
  if(_pyunit_WORKING_DIRECTORY)
    set(_chdir_prefix "bash -c \"cd ${_pyunit_WORKING_DIRECTORY} && ")
    set(_chdir_suffix "\"")
  endif()

  # Check that the file exists, #1621
  set(_file_name _file_name-NOTFOUND)
  if(IS_ABSOLUTE ${file})
    set(_file_name ${file})
  else()
    find_file(_file_name ${file}
              PATHS ${CMAKE_CURRENT_SOURCE_DIR}
              NO_DEFAULT_PATH
              NO_CMAKE_FIND_ROOT_PATH)  # for cross-compilation.  thanks jeremy.
    if(NOT _file_name)
      message(FATAL_ERROR "Can't find pyunit file \"${file}\"")
    endif()
  endif()

  # We look for ROS_TEST_COVERAGE=1
  # to indicate that coverage reports are being requested.
  if("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")
    set(_covarg "--cov")
  else()
    set(_covarg)
  endif()

  # Create a legal test name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname ${file})
  # We use rostest to call the executable to get process control, #1629
  rospack_append_test_to_cache(catkin-tests "${_chdir_prefix}${ROSUNIT_EXE} --name=${_testname} --time-limit=${_pyunit_TIMEOUT} --package=${PROJECT_NAME} -- ${_file_name} ${_covarg}${_chdir_suffix}")
  rospack_append_test_to_cache(catkin-tests "${ROSUNIT_CHECK_TEST_RAN_EXE} ${CMAKE_BINARY_DIR}/test_results/${PROJECT_NAME}/TEST-${_testname}.xml")
endfunction()

function(rospack_add_gtest exe)

  # Look for optional TIMEOUT argument, #2645
  parse_arguments(_gtest "TIMEOUT;WORKING_DIRECTORY" "" ${ARGN})
  if(NOT _gtest_TIMEOUT)
    set(_gtest_TIMEOUT 60.0)
  endif()
  if(_gtest_WORKING_DIRECTORY)
    set(_chdir_prefix "bash -c \"cd ${_gtest_WORKING_DIRECTORY} && ")
    set(_chdir_suffix "\"")
  endif()

  # Create the program, with basic + gtest build flags
  find_package(GTest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIRS})
  add_executable(${exe} EXCLUDE_FROM_ALL ${_gtest_DEFAULT_ARGS})
  target_link_libraries(${exe} ${GTEST_LIBRARIES} ${THREADS_LIBRARY})

  # Make sure the executable is built before running tests
  add_dependencies(tests ${exe})

  # Create a legal test name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname ${exe})
  get_target_property(_exe_path ${exe} RUNTIME_OUTPUT_DIRECTORY)
  # We use rosunit to call the executable to get process control, #1629, #3112
  rospack_append_test_to_cache(catkin-tests "${_chdir_prefix}${_exe_path}/${exe} --gtest_output=xml:$ENV{ROS_TEST_RESULTS_DIR}/${PROJECT_NAME}/TEST-${exe}.xml${_chdir_suffix}")
endfunction()

include_directories(${PROJECT_SOURCE_DIR}/src)
rospack_add_gtest(${PROJECT_NAME}-utest 
          test/utest.cpp ${PROJECT_SOURCE_DIR}/src/rospack_cmdline.cpp ${PROJECT_SOURCE_DIR}/src/utils.cpp
          WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(${PROJECT_NAME}-utest rospack)

# Prepare to run the tests.  This could be cleaner.
add_custom_target(${PROJECT_NAME}-prepare_test
                  COMMAND cmake -E copy_directory ${PROJECT_SOURCE_DIR}/test ${CMAKE_CURRENT_BINARY_DIR}
                  COMMAND cmake -E chdir test/deep python deep.py)
add_dependencies(tests ${PROJECT_NAME}-prepare_test)

message("TODO: enable rospack's pyunit tests")
#configure_file(test/utest.py.in
#               ${CMAKE_CURRENT_BINARY_DIR}/test/utest.py)
#rospack_add_pyunit(${CMAKE_CURRENT_BINARY_DIR}/test/utest.py
#           WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
#configure_file(test/utest_rosstack.py.in
#               ${CMAKE_CURRENT_BINARY_DIR}/test/utest_rosstack.py)
#rospack_add_pyunit(${CMAKE_CURRENT_BINARY_DIR}/test/utest_rosstack.py
#           WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
