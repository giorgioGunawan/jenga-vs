# CMake generated Testfile for 
# Source directory: /home/matt/JBOT_Working/catkin_ws/src/rviz_visual_tools
# Build directory: /home/matt/JBOT_Working/catkin_ws/build/rviz_visual_tools
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_rviz_visual_tools_rostest_launch_rvt_test.launch "/home/matt/JBOT_Working/catkin_ws/build/rviz_visual_tools/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/matt/JBOT_Working/catkin_ws/build/rviz_visual_tools/test_results/rviz_visual_tools/rostest-launch_rvt_test.xml" "--return-code" "/usr/bin/python2 /opt/ros/melodic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/matt/JBOT_Working/catkin_ws/src/rviz_visual_tools --package=rviz_visual_tools --results-filename launch_rvt_test.xml --results-base-dir \"/home/matt/JBOT_Working/catkin_ws/build/rviz_visual_tools/test_results\" /home/matt/JBOT_Working/catkin_ws/src/rviz_visual_tools/launch/rvt_test.launch ")
subdirs("gtest")
