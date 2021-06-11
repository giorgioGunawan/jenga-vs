execute_process(COMMAND "/home/matt/JBOT_Working/catkin_ws/build/robotiq_3f_gripper_control/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/matt/JBOT_Working/catkin_ws/build/robotiq_3f_gripper_control/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
