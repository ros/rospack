^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rospack
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.23 (2014-02-25)
-------------------
* only perform backquote substitution when needed (`#34 <https://github.com/ros/rospack/issues/34>`_)

2.1.22 (2014-01-07)
-------------------
* use specific python version catkin has decided on (`#29 <https://github.com/ros/rospack/issues/29>`_)
* python 3 compatibility (`#25 <https://github.com/ros/rospack/issues/25>`_, `#27 <https://github.com/ros/rospack/issues/27>`_)
* fall back gracefully whe gtest is not available
* update package urls

2.1.21 (2013-07-05)
-------------------
* honor CATKIN_IGNORE marker file when crawling for packages (`#21 <https://github.com/ros/rospack/issues/21>`_)

2.1.20 (2013-07-03)
-------------------
* improve error message to include package names when circular dependency is detected (`#18 <https://github.com/ros/rospack/issues/18>`_)
* check for CATKIN_ENABLE_TESTING to enable configure without tests
* add '-h' option

2.1.19 (2013-06-06)
-------------------
* modified command 'list-duplicates' to output the paths where the packages were found (`#3 <https://github.com/ros/rospack/issues/3>`_)
* modified 'rospack plugins' to not use rosdep (`#5 <https://github.com/ros/rospack/issues/5>`_)
* improve Windows support  (`#10 <https://github.com/ros/rospack/issues/10>`_)
* use find_package() for tinyxml (if available)

2.1.18 (2013-03-21)
-------------------
* invert order of package type detection (dry before wet) (`ros-infrastructure/rospkg#30 <https://github.com/ros-infrastructure/rospkg/issues/30>`_)

2.1.17 (2013-03-08)
-------------------
* output full pkg-config command in case of errors (`#8 <https://github.com/ros/rospack/issues/8>`_)
* handle None as return value for call_pkg_config (`#8 <https://github.com/ros/rospack/issues/8>`_)
* fix crawling to always recrawl when forced (`#9 <https://github.com/ros/rospack/issues/9>`_)

2.1.16 (2013-01-13)
-------------------
* fix segfault for command depends1 which ignores exceptions and calls isSysPackage again (`#4 <https://github.com/ros/rospack/issues/4>`_)

2.1.15 (2012-12-06)
-------------------
* first public release for Groovy
