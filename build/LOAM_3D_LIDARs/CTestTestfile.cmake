# CMake generated Testfile for 
# Source directory: /home/derm/ROS/src/LOAM_3D_LIDARs
# Build directory: /home/derm/ROS/build/LOAM_3D_LIDARs
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_loam_ouster_rostest_test_loam.test "/home/derm/ROS/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/derm/ROS/build/test_results/loam_ouster/rostest-test_loam.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/derm/ROS/src/LOAM_3D_LIDARs --package=loam_ouster --results-filename test_loam.xml --results-base-dir \"/home/derm/ROS/build/test_results\" /home/derm/ROS/build/LOAM_3D_LIDARs/test/loam.test ")
set_tests_properties(_ctest_loam_ouster_rostest_test_loam.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/derm/ROS/src/LOAM_3D_LIDARs/CMakeLists.txt;67;add_rostest;/home/derm/ROS/src/LOAM_3D_LIDARs/CMakeLists.txt;0;")
subdirs("src/lib")
