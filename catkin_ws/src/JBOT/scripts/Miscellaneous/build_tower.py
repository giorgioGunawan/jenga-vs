#! /usr/bin/env python

# Author: Quokka robotics
# Description: This script programs that UR5 robotic arm to build a tower of blocks
# that are generated in the simulation environment. It also includes obstacle detection
# to avoid knocking over the tower during constrcution.

import rospy
import geometry_msgs.msg
from gazebo_msgs.srv import GetModelState
from move_group_interface import MoveGroupPythonInteface
from std_msgs.msg import Float32MultiArray
import tf as tf
import math

# Global list containing the string ID's of the blocks and goal position
names = ["block_1", "block_2", "block_3", "block_4", "block_5", "goal"]


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


# For ROS subscriber
def callback(input_msg):
    print(input_msg)
    return input_msg


def set_up_tower_build_sim():
    """
    Randomises the positions of the blocks and initialises the move group interface
    :returns: The created instance of the move group interface
    """
    try:
        print("Setting up the move group interface to the robot...")
        created_mgpi = MoveGroupPythonInteface()
        print("Simulation environment is good to go!")
        return created_mgpi
    except rospy.ROSInterruptException:
        print("ROS was interrupted suddenly!")
        return
    except KeyboardInterrupt:
        print("Keyboard interrupt!")
        return


def list_model_states(model_arr):
    """
    Lists the models with their Pose
    :param positions: Takes in a list of the models stored as objects
    """
    try:
        # Loop through and print all model information
        for i in model_arr:
            # Print the model state
            print("Model ID: " + i.model_id)
            print("Model Position (relative to the world frame):")
            print("x = " + str(i.pose.position.x) + " y = " + str(i.pose.position.y) + " z = " + str(i.pose.position.z))
            print("\n")
    except rospy.ROSInterruptException:
        print("ROS was interrupted suddenly!")
        return
    except KeyboardInterrupt:
        print("Keyboard interrupt!")
        return


def get_block_positions():
    """
    Obtains the coordinates of the blocks from the simulation using the gazebo ros service
    :returns: The array containing the objects of each block
    """
    try:
        print("Getting the block positions and goal positions...")

        # Retrieve the state information for each module in the simulation
        model_states = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        block_positions = []

        # Loops through the global names array containing the model ID's
        # and creates an object for each, storing them in the block positions list
        for i in names:
            # Get model pose and twist (e.g. for block_1 only)
            model_to_extract = model_states(i, "world")

            # Create a new model object and assign it the correct name
            new_model = model(i)

            # Assign the new model the coordinates extracted from the /gazebo/model_state topic
            new_model.assign_coordinates(model_to_extract)

            # Add it to the array of blocks
            block_positions.append(new_model)


        # Return the resultant list
        return block_positions
    except rospy.ROSInterruptException:
        print("ROS was interrupted suddenly!")
        return
    except KeyboardInterrupt:
        print("Keyboard interrupt!")
        return


def move_to_target(mgpi, block, height):
    """
    Move the arm to the desired target pose (cartesian coordinates for the end effector)
    :param mgpi: The previously initialised move group object
    :param x: Target x coordinate
    :param y: Target y co-ordinate
    :param z: Target z co-ordinate
    :param rotx: Target roll angle
    :param roty: Target pitch angle
    :param rotz: Target yaw angle
    """
    try:
        # Convert the Euler angles from the /gazebo/model_state topic into a target quaternion (to make sure the gripper is always pointing down)
        target_quat = tf.transformations.quaternion_from_euler(math.pi/2, math.pi/2, 0)

        # Set the target pose (quaternions used for orientation! hence the conversion in the previous line of code to convert from euler angles)
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = round(block.pose.position.x, 5)
        target_pose.position.y = round(block.pose.position.y, 5)
        target_pose.position.z = height  # Constant as set by the assignment (accounting for simulation)
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

def add_block_tower(mgpi, block, goal, tower_height):
    """
    Moves the arm into a pose to get specified block, pick it up
    and add it to the tower
    :param mgpi: The previously initialised move group object to operate the arm
    :param block: The model object for the chosen block
    :param block_id: The model object for the goal position
    :param tower_height: The height at which the tower stands
    """
    try:

        # Set clearance height so arm does not hit tower when moving (To avoid collision with the tower)
        clearance_height = 0.5

        # Set the height to pick up the block from (THIS WILL HAVE TO BE ADJUSTED WHEN USING THE REAL HARDWARE)
        pick_up_height = 0.086

        # Move the arm to the block position (account for the offset by setting z = 0.086m)
        move_to_target(mgpi, block, clearance_height)

        # Move the arm down to pick up the block
        move_to_target(mgpi, block, pick_up_height)

        # Close the gripper
        mgpi.open_gripper()
        mgpi.close_gripper(block.model_id)

        # Move the arm back to the clearance height above the tower to avoid collisions
        move_to_target(mgpi, block, clearance_height)

        # Move the block to a point above the goal position
        move_to_target(mgpi, goal, clearance_height)

        # Place the block down on the tower by lowering it down to teh current tower height
        move_to_target(mgpi, goal, tower_height)

        # Let go of the block
        mgpi.open_gripper(block.model_id)

        # move directly above the goal position to avoid further collision
        move_to_target(mgpi, goal, clearance_height)
        return
    except rospy.ROSInterruptException:
        print("ROS was interrupted suddenly!")
        return
    except KeyboardInterrupt:
        print("Keyboard interrupt!")
        return


def tower_build(mgpi, model_list):
    """
    Builds the tower using the information previously extracted from the ros service call
    :param mgpi: The previously initialised move group object to operate the arm
    :param models: The list containing model objects with the pose and twist for each block and the goal position
    """
    try:
        # Moves the arm into the home position
        print("Moving the arm into the home position...\n")
        mgpi.move_to_joint_state(mgpi.home_joint_angles)
        print("Done! The robot is now in the home position\n")

        # Prints the state of the robot
        print("******************************************\n")
        print("The current state of the robot is:\n")
        mgpi.print_robot_state()
        print("Done!")

        # Sets the height of each block
        block_height = 0.086

        # Sets the height of the tower to start at 1 block (the height to drop each block)
        tower_height = block_height

        # Loops through the blocks and places them on the tower one by one. Increases the block height each time
        for i in range(len(names) - 1):
            add_block_tower(mgpi, model_list[i], model_list[len(model_list) - 1], tower_height)
            tower_height += block_height


        # Reports task completion
        print("**************************************************************************")
        print("The tower is built! (Hopefully)")
        print("**************************************************************************")
        return
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
        raw_input("Press Enter to set up tower building simulation...")

        # Set up the tower simulation!
        mgpi = set_up_tower_build_sim()

        raw_input("Press Enter to get the model states and print them to the screen...")

        # Retrieve the positions of the blocks by subscribing to the
        # topic /gazebo/model_states
        model_list = get_block_positions()

        print("Model states obtained! Printing to the screen...")

        # List the model states
        list_model_states(model_list)

        raw_input("Press enter to build the tower!")

        # Builds the tower
        tower_build(mgpi, model_list)

        raw_input("Press Enter to return the arm to the home position!")

        # Return the arm to home position once the tower is built
        mgpi.move_to_joint_state(mgpi.home_joint_angles)

    except rospy.ROSInterruptException:
        print("ROS was interrupted suddenly!")
        return
    except KeyboardInterrupt:
        print("Keyboard interrupt!")
        return

# Main function (program entry point)
if __name__ == '__main__':
    main()

