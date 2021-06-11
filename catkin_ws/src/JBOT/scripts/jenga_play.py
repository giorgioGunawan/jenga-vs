#! /usr/bin/env python

# Author: Poke test 
# Description: This script programs that UR5e arm with hardcoded coordinates
# to reomve the block and pick it up from the other side

# Import Python modules
import rospy
import geometry_msgs.msg
from move_group_interface import MoveGroupPythonInteface
from visual_servoyer import VisualServoyer
import tf as tf
import math
import copy

# Import ROS mesages types
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist

# Define a global variable to keep track of the latest visual servoing test

class model:
    """
    Model class that stores the Pose and Twist of each model in the simulation
    Makes sure the gazebo msg information is stored a geometry msg for the move group interface
    """

    def __init__(self, name):
        """
        (Constructor) Set up class variables
        :param model_id: stores the name of the model as a string
        """
        self.model_id = name
        self.pose = geometry_msgs.msg.Pose()
        self.twist = geometry_msgs.msg.Twist()
        return

    def assign_coordinates(self, model_state):
        """
        Assigns the Pose and positions to the model object using the model state from the service
        :param model_state: has the model state information retrieved for a specific model obtained from the ros service
        """
        print("Model state being assigned for block: " + self.model_id + "\n")
        self.pose = model_state.pose
        self.twist = model_state.twist
        return


def vs_callback(data):
    """
    Defines a callback function for the visual servoing
    velocity feedback values
    :param data (type:TwistStamped): is the twist message from the
    visual servoing module 
    """
    print("Visual servoing recieved")


def move_to_target(mgpi, x, y, z, pose):
    """
    Move the arm to the desired target pose (cartesian coordinates for the end effector)
    :param mgpi (Type: MoveGroupInteface object): The previously initialised move group object
    :param x: Target x coordinate
    :param y: Target y co-ordinate
    :param z: Target z co-ordinate
    :param pose: Direction the gripper is facing
    """
    try:
        # Convert the Euler angles to quaternion based on input pose
        if pose == "down_across":
            target_quat = tf.transformations.quaternion_from_euler(0, math.pi / 2, 0)
        elif pose == "down_straight":
            target_quat = tf.transformations.quaternion_from_euler(math.pi / 2, math.pi / 2, 0)
        elif pose == "right_horz":
            target_quat = tf.transformations.quaternion_from_euler(math.pi / 2, 0, 0)
        elif pose == "right_vert":
            target_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        elif pose == "left_horz":
            target_quat = tf.transformations.quaternion_from_euler(-math.pi / 2, math.pi, 0)
        elif pose == "left_vert":
            target_quat = tf.transformations.quaternion_from_euler(0, math.pi, 0)
        elif pose == "forward_horz":
            target_quat = tf.transformations.quaternion_from_euler(math.pi / 2, 0, math.pi / 2)
        elif pose == "forward_vert":
            target_quat = tf.transformations.quaternion_from_euler(0, 0, math.pi / 2)
        elif pose == "back_horz":
            target_quat = tf.transformations.quaternion_from_euler(math.pi / 2, 0, -math.pi / 2)
        elif pose == "back_vert":
            target_quat = tf.transformations.quaternion_from_euler(0, 0, -math.pi / 2)
        else:
            target_quat = tf.transformations.quaternion_from_euler(0, 0, 0)

        # Set the target pose (quaternions used for orientation! hence the conversion in the previous line of code to convert from euler angles)
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.x = round(target_quat[0], 5)
        target_pose.orientation.y = round(target_quat[1], 5)
        target_pose.orientation.z = round(target_quat[2], 5)
        target_pose.orientation.w = round(target_quat[3], 5)

        # Move to the specified target position
        mgpi.move_eef_to_pose(target_pose)
        return
    except rospy.ROSInterruptException:
        print("ROS was interrupted suddenly!")
        return
    except KeyboardInterrupt:
        print("Keyboard interrupt!")
        return

    mgpi.move_eef_to_pose(target_pose)

def linear_waypoint_path(mgpi,x, y, z, precision):
    # print("Press Enter to continue..")
    # raw_input()

    waypoints = []
    wpose = mgpi.get_current_eef_pose().pose
    wpose.position.x += x
    wpose.position.y += y
    wpose.position.z += z
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    plan, fraction = mgpi.plan_cartesian_path(waypoints, precision, 0.0, 200)

    # mgpi.display_trajectory(plan)

    mgpi.execute_plan(plan)

def move_to_block(mgpi, row, col, side, action, block):

    # Current height of arm
    z = 0.4

    if side == "left":
        # set direction of movement
        dir = 1
    elif side == "right":
        # set direction of movement
        dir = -1

    # Set the name of the block to attach to
    if block == 1:
        block_name = "block_g_2"
    else:
        block_name = "block_e_1"

    # move to correct row
    base_height = 0
    block_height = 0.02
    row_height = base_height + block_height * (row + 1)

     # Precision to change column (in m)
    precision_row = 0.001

    # move to correct column
    # Distance to change column (in m)
    distance_col = 0.025

    # Precision to change column (in m)
    precision_col = 0.0001

    if block == 1:
        if side == "left":
            # move above left side
            mgpi.move_to_joint_state([5.631824993553783, -2.355252859218327, -0.6936486975855338, 3.0496427718611017, -5.36301877051246, -3.141894596714735])  # values are in radians

            linear_waypoint_path(mgpi, -0.025, 0, 0, 0.001)


        elif side == "right":
            # move above right side
            mgpi.move_to_joint_state([0.6512924061554468, -0.7860350076551637, 0.6936477048767058, 0.09268269988641631, -0.9201223814302972, 3.141147584974476])  # values are in radians

            linear_waypoint_path(mgpi, 0.025, 0, 0, 0.001)

        # go across at 0.1mm steps
        if col == 0:
            linear_waypoint_path(mgpi, 0, distance_col*dir, 0, precision_col)
        elif col == 2:
            linear_waypoint_path(mgpi, 0, -distance_col*dir, 0, precision_col)

        # go down at 1mm steps
        linear_waypoint_path(mgpi, 0, 0, -(z - row_height), precision_row)

    print("start servoing")
    servo_waypoint_path(mgpi, side, block)

    # Push or pull block
    # Distance to push and pull (in m)
    distance_push = 0.11
    distance_pull = 0.12

    # Precision to push and pull (in m)
    precision_pp = 0.001

    if action == "push":
        # push block
        linear_waypoint_path(mgpi, distance_push*dir, 0, 0, precision_pp)
        linear_waypoint_path(mgpi, -distance_push*dir, 0, 0, precision_pp)
    elif action == "pull":
        # pull
        # open gripper
        mgpi.open_gripper()

        # move to block
        linear_waypoint_path(mgpi, distance_pull*dir, 0, 0, precision_pp)

        # close gripper
        mgpi.close_gripper(block_name)

        # pull block out
        linear_waypoint_path(mgpi, -distance_pull*dir, 0, 0, precision_pp)

        # open gripper
        mgpi.open_gripper(block_name)


    if block == 2:

        # go up at 1mm steps
        linear_waypoint_path(mgpi, 0, 0, (z - row_height), 0.001)


def vs_test(mgpi):

    # Distance to move (in m)
    distance = 0.05

    # Precision to move (in m)
    precision = 0.0001

    ## MOVE RIGHT AND LEFT RELATIVE TO RIGHT WALL
    # go forward 0.05m at 1mm steps
    linear_waypoint_path(mgpi, 0, distance,0 , precision)
    # go back 0.05m at 1mm steps
    linear_waypoint_path(mgpi, 0, -distance, 0, precision)

    # go back 0.05m at 1mm steps
    linear_waypoint_path(mgpi, 0, -distance, 0, precision)
    # go forward 0.05m at 1mm steps
    linear_waypoint_path(mgpi, 0, distance, 0, precision)

    ## MOVE UP AND DOWN RELATIVE TO RIGHT WALL
    # go up 0.05m at 1mm steps
    linear_waypoint_path(mgpi, 0, 0, distance, precision)
    # go down 0.05m at 1mm steps
    linear_waypoint_path(mgpi, 0, 0, -distance, precision)

    # go down 0.05m at 1mm steps
    linear_waypoint_path(mgpi, 0, 0, -distance, precision)
    # go up 0.05m at 1mm steps
    linear_waypoint_path(mgpi, 0, 0, distance, precision)

    ## MOVE BACKWARDS AND FORWARDS RELATIVE TO RIGHT WALL
    # go right 0.05m at 1mm steps
    linear_waypoint_path(mgpi, -distance, 0, 0, precision)
    # go left 0.05m at 1mm steps
    linear_waypoint_path(mgpi, distance, 0, 0, precision)

    # go left 0.05m at 1mm steps
    linear_waypoint_path(mgpi, distance, 0, 0, precision)
    # go right 0.05m at 1mm steps
    linear_waypoint_path(mgpi, -distance, 0, 0, precision)


def apply_visual_servoing(mgpi):
    """
    Function to align the arm with the block to either push it or pull it
    :param mgpi (type: MoveGroupInteface object): The move group interface for the
    UR5e arm
    """
    try:
        print('Servoing the block...')
        
        # subscriber = rospy.Subscriber("/servoing_results", Twist, vs_callback)
        # rospy.spin()
        i = 0
        while i < 10:
            msg = rospy.wait_for_message("/servoing_results1", Twist)
            print(msg)
            i += 1

    except Exception as e:
        print(e)
        print("Error in Visual Servoing")

def servo_waypoint_path(mgpi,side,block):

    # # command velocity from visual servoying node
    # vel = geometry_msgs.msg.Twist()
    # vel.linear.x = 0
    # vel.linear.y = 0.1
    # vel.linear.z = 0.1
    # vel.angular.x = 0
    # vel.angular.y = 0
    # vel.angular.z = 0


    precision = 0.01
    max_linear_translation = 0.01
    threshold = 0.0002

    count = 0
    while True:
        # call service
        # wait for service
        if block == 1:
            vel = rospy.wait_for_message("/servoing_results1", Twist)
        elif block == 2:
            vel = rospy.wait_for_message("/servoing_results2", Twist)

        print(vel)

        max_linear = abs(max(vel.linear.x, vel.linear.y, vel.linear.z))

        if max_linear < threshold:
            count +=1

            if count > 3:
                print("break")
                break

        if max_linear > max_linear_translation:
            print("overmax linear")

            dx = float(vel.linear.x) * max_linear_translation / max_linear
            dy = float(vel.linear.y) * max_linear_translation / max_linear
            dz = float(vel.linear.z) * max_linear_translation / max_linear

        else:
            dx = vel.linear.x
            dy = vel.linear.y
            dz = vel.linear.z


        print("scaled velocity")
        print(dx)
        print(dy)
        print(dz)

        if side == "left":
            linear_waypoint_path(mgpi, dz, -dx, dy, precision)
        elif side == "right":
            linear_waypoint_path(mgpi, -dz, dx, dy, precision)

def poker(mgpi, poker, action):

    poker = model("poker")
    # Hardcode the poker position
    poker.pose.position.x = 0.25
    poker.pose.position.y = 0.65
    poker.pose.position.z = 0.220

    # Move above the poker
    move_to_target(mgpi,
                   poker.pose.position.x,
                   poker.pose.position.y,
                   poker.pose.position.z,
                   "forward_horz")

    # Move down to pick up the poker
    linear_waypoint_path(mgpi, 0, 0, -0.018, 0.0001)

    if action == "pick":
        # Pick up the poker
        mgpi.close_gripper(poker.model_id)
    elif action == "drop":
        # Drop the poker
        mgpi.open_gripper(poker.model_id)

    # Move up
    linear_waypoint_path(mgpi, 0, 0, 0.018, 0.0001)


def play_jenga(mgpi):
    """
    Function to poke a block out and pick it up from the other side
    :param mgpi (type: MoveGroupInteface object): The move group interface for the
    UR5e arm
    """
    # Try to execute the action with the arm
    try:
        print("Attempting to remove block...")

        # pick up the poker
        poker(mgpi, poker, "pick")

        # Push block 1
        move_to_block(mgpi, 1, 1, "left", "push", 1)

        # Push block 2
        move_to_block(mgpi, 1, 1, "left", "push", 2)

        # drop poker on platform
        poker(mgpi, poker, "drop")

        # pull block 1
        move_to_block(mgpi, 1, 1, "right", "pull", 1)

        # pull block 2
        move_to_block(mgpi, 1, 1, "right", "pull", 2)

        # pick up the poker
        poker(mgpi, poker, "pick")

        # mgpi.move_to_joint_state([5.631824993553783, -2.355252859218327, -0.6936486975855338, 3.0496427718611017, -5.36301877051246, -3.141894596714735])  # values are in radians
        #
        # linear_waypoint_path(mgpi, 0, 0, -0.35, 0.01)
        # # vs_test(mgpi)
        # servo_waypoint_path(mgpi)
        
        
        # # row and column of block that needs to be removed
        # target_block_row = 1
        # target_block_col = 1
        #
        # raw_input("[INFO] Press Enter to push target block...")
        #
        # move_to_block(mgpi, target_block_row, target_block_col, "left", "push")
        #
        # # Attempt to visual servo the block
        # apply_visual_servoing(mgpi)


        #raw_input("[INFO] Press Enter to pull out target block...")

        #move_to_block(mgpi, target_block_row, target_block_col, "right", "pull")



        

    except rospy.ROSInterruptException:
        print("ROS was interrupted suddenly!")
        return
    except KeyboardInterrupt:
        print("Keyboard interrupt!")
        return

def main():
    """
    Main function called during the script execution
    """
    try:
        # # Set up a subscriber to the visual servoing test
        # rospy.init_node("jenga_player", anonymous=True)
        raw_input("[INFO] Press Enter to try removing a jenga block...")

        # Create a move group interface with the arm!
        mgpi = MoveGroupPythonInteface()

        # # Return the arm to home position once the tower is built
        # mgpi.move_to_joint_state(mgpi.home_joint_angles)

        # Attempt to remove a block
        play_jenga(mgpi)

        # Report completion
        print("[INFO] Poke test simulation complete! ( Hopefully worked... :) )")
    except rospy.ROSInterruptException:
        print("ROS was interrupted suddenly!")
        return
    except KeyboardInterrupt:
        print("Keyboard interrupt!")
        return

# Main function (program entry point)
if __name__ == '__main__':
    main()

