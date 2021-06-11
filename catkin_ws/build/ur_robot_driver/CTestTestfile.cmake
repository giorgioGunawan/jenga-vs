# CMake generated Testfile for 
# Source directory: /home/matt/JBOT_Working/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver
# Build directory: /home/matt/JBOT_Working/catkin_ws/build/ur_robot_driver
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_ur_robot_driver_rostest_test_driver.test "/home/matt/JBOT_Working/catkin_ws/build/ur_robot_driver/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/matt/JBOT_Working/catkin_ws/build/ur_robot_driver/test_results/ur_robot_driver/rostest-test_driver.xml" "--return-code" "/usr/bin/python2 /opt/ros/melodic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/matt/JBOT_Working/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver --package=ur_robot_driver --results-filename test_driver.xml --results-base-dir \"/home/matt/JBOT_Working/catkin_ws/build/ur_robot_driver/test_results\" /home/matt/JBOT_Working/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/test/driver.test ")
subdirs("gtest")
