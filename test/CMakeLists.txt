include_directories(${PROJECT_SOURCE_DIR}/src)
add_gtest(${PROJECT_NAME}-utest test/utest.cpp ${PROJECT_SOURCE_DIR}/src/rospack_cmdline.cpp ${PROJECT_SOURCE_DIR}/src/utils.cpp)
target_link_libraries(${PROJECT_NAME}-utest rospack)

add_custom_target(${PROJECT_NAME}-copy_test_files
                  COMMAND cmake -E copy_directory ${PROJECT_SOURCE_DIR}/test ${CMAKE_CURRENT_BINARY_DIR})
add_custom_target(${PROJECT_NAME}-make_deep_directories
                  COMMAND cmake -E chdir test/deep python deep.py)
add_dependencies(${PROJECT_NAME}-make_deep_directories ${PROJECT_NAME}-copy_test_files)
add_dependencies(tests ${PROJECT_NAME}-make_deep_directories)

add_nosetests(test)
