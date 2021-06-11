# CMake generated Testfile for 
# Source directory: /home/matt/JBOT_Working/catkin_ws/src/baldor
# Build directory: /home/matt/JBOT_Working/catkin_ws/build/baldor
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_baldor_nosetests_tests "/home/matt/JBOT_Working/catkin_ws/build/baldor/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/matt/JBOT_Working/catkin_ws/build/baldor/test_results/baldor/nosetests-tests.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/matt/JBOT_Working/catkin_ws/build/baldor/test_results/baldor" "/usr/bin/nosetests-2.7 -P --process-timeout=60 --where=/home/matt/JBOT_Working/catkin_ws/src/baldor/tests --with-xunit --xunit-file=/home/matt/JBOT_Working/catkin_ws/build/baldor/test_results/baldor/nosetests-tests.xml")
subdirs("gtest")
