# MTRX5700 MP JBOT
### Running the demo
#### In simulation
Open 5 terminals and source ROS commands in all of them.
```bash
source the bash file in catkin_ws
```
1. Terminal 2 - Launch the simulated environment in gazebo
```bash
roslaunch JBOT JBOT_arm.launch
```
2. Terminal 3 - Launch the MoveIt! Planner and Controller **with sim:=true**. Wait until you see a green text saying *You can start planning now!*
```bash
roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch sim:=true
```
3. Terminal 4 - Launch RViz to visualize the planning scene **with config:=true**
```bash
roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true
```
4. Terminal 5
'''bash
rosrun JBOT jenga_play.py (To pick up the tool to poke bricks out with)
```
5. Terminal 6
'''bash
rosrun JBOT visual_servoyer1.py 
```
6. Terminal 7
'''bash
rosrun JBOT visual_servoyer2.py 
```
#### On Hardware
Open 5 terminals and source ROS commands in all of them.
```bash
source ur5espace/devel/setup.bash
```
1. Terminal 1 - roscore
```bash
roscore
```
2. Terminal 2 - Launch the robot driver with the correct IP, kinematic configuration and **limited:=true**. Check with your tutor for the correct IP.
```bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.10 limited:=true kinematics_config:=~/ur5espace/src/universal_robot/ur_e_description/config/ur5e_calib.yaml
```
3. Terminal 3 - Launch the MoveIt! Planner and Controller **with sim:=false**. Wait until you see a green text saying *You can start planning now!*
```bash
roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch sim:=false
```
4. Terminal 4 - Launch RViz to visualize the planning scene **with config:=true**
```bash
roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true
```
5. Terminal 5 - Launch the demo (or your code when ready)
```bash
rosrun assignment_1 demo.py
```
or
```bash
rosrun assignment_1 build_tower.py
```


