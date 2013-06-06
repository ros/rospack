^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rospack
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.19 (2013-06-06)
-------------------
* modified command 'list-duplicates' to output the paths where the packages were found (`#3 <https://github.com/ros/rospack/issues/3>`_)
* modified 'rospack plugins' to not use rosdep (`#5 <https://github.com/ros/rospack/issues/5>`_)
* improve Windows support  (`#10 <https://github.com/ros/rospack/issues/10>`_)
* use find_package() for tinyxml (if available)

2.1.18 (2013-03-21)
-------------------
* invert order of package type detection (dry before wet) (`ros/rospkg#30 <https://github.com/ros/rospkg/issues/30>`_)

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
