# CMake generated Testfile for 
# Source directory: /home/matt/JBOT_Working/catkin_ws/src/criutils
# Build directory: /home/matt/JBOT_Working/catkin_ws/build/criutils
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_criutils_nosetests_tests "/home/matt/JBOT_Working/catkin_ws/build/criutils/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/matt/JBOT_Working/catkin_ws/build/criutils/test_results/criutils/nosetests-tests.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/matt/JBOT_Working/catkin_ws/build/criutils/test_results/criutils" "/usr/bin/nosetests-2.7 -P --process-timeout=60 --where=/home/matt/JBOT_Working/catkin_ws/src/criutils/tests --with-xunit --xunit-file=/home/matt/JBOT_Working/catkin_ws/build/criutils/test_results/criutils/nosetests-tests.xml")
subdirs("gtest")
