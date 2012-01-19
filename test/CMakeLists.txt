message("TODO turn rospack's tests back on, after making them not depend on
rosunit")

#include_directories(${PROJECT_SOURCE_DIR}/src)
#add_gtest(${PROJECT_NAME}-utest 
#          test/utest.cpp ${PROJECT_SOURCE_DIR}/src/rospack_cmdline.cpp ${PROJECT_SOURCE_DIR}/src/utils.cpp
#          WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
#target_link_libraries(${PROJECT_NAME}-utest rospack)
#
## Prepare to run the tests.  This could be cleaner.
#add_custom_target(${PROJECT_NAME}-prepare_test
#                  COMMAND cmake -E copy_directory ${PROJECT_SOURCE_DIR}/test ${CMAKE_CURRENT_BINARY_DIR}
#                  COMMAND cmake -E chdir test/deep python deep.py)
#add_dependencies(tests ${PROJECT_NAME}-prepare_test)
#configure_file(test/utest.py.in
#               ${CMAKE_CURRENT_BINARY_DIR}/test/utest.py)
#add_pyunit(${CMAKE_CURRENT_BINARY_DIR}/test/utest.py
#           WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
#configure_file(test/utest_rosstack.py.in
#               ${CMAKE_CURRENT_BINARY_DIR}/test/utest_rosstack.py)
#add_pyunit(${CMAKE_CURRENT_BINARY_DIR}/test/utest_rosstack.py
#           WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
