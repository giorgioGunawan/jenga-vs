#! /usr/bin/env python
#
# Author: Quokka Robotics
# Description: This program takes in a frame of an image and determines the correct velocity to move the robot arm by outputting a vector of size 6
# with the velocities in the x, y and z direction as well as the angular velocities around the x, y and z axes

import roslib
import rospy
import sys
import numpy as np
import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
import math

# Import the message type from existing packages
from sensor_msgs.msg import CompressedImage, CameraInfo
from sensor_msgs.msg import Image as Image_msg
from geometry_msgs.msg import Twist



class VisualServoyer(object):
    """
    Class to control the gripper
    """
    def __init__(self):
        """
        Defines a constructor for the Visual Servoyer and
        defines the class variables
        """
        print("[INFO] Visual Servoyer created!")

        # Class variables
        self.publisher = None          # Landmark finder publisher
        self.cam_info_sub = None       # Landmark finder subscriber
        self.cam_colour_sub = None     # Subsrciber to camera image
        self.cam_dep_sub = None        # Subscriber to the camera depth topic
        self.depth_img = None          # Stores the depth img for current depth frame
        self.masks = None              # Stores the HSV ranges for each landmark colour
        self.labels = None             # Keeps track of the labels in the frame
        self.K = None                  # Defines the Camera Matrix

        # set mode to determine if the visual servoying is configured for pushing or pulling
        self.mode = "push"

        # number of feature points to match
        self.num_of_points = 3

        # Hard code 3 points on the screen to represent the desired distance from the robot (Measure this with the actual jenga set)
        self.ideal_features = np.array([270, 392, 285, 372, 300, 392]).reshape((6,1))

        # feature location from camera image
        self.cam_features = np.array([1,2,3,4,5,6]).reshape((6,1))

        # flag checking if target has been reached
        self.target_reached = False

        # EQUATION: vel = -lambda * inv(L) * error

        # velocity matrix
        self.vel = np.zeros((6,1))

        # Gain on controller, essentially sets arm speed, although too high of a value will cause the
        # function to diverge.
        self.gain_lambda = 0.5

        # image jacobian matrix
        self.jacobian_matrix = np.zeros((2*3,6))

        # error matrix
        self.error = np.zeros((6,1))

    def show_image(self, image, window):
        """
        Function to show the CV image to the screen
        :param: Function to show the CV image to screen
        :param: window is the name of the imshow window for reference
        """
        cv2.imshow(window, image)
        cv2.waitKey(1)
        return

    def camera_colour_callback(self, img):
        """
        Defines a function to look for 3 coloured points  in a camera frame
        obtained by subscribing to the camera topic
        :param img: is the ROS RGB image from real sense camera
        """
        print("[INFO] Image recieved")
        try:
            # Convert the CompressedImage msg to a cv image
            # matrix for processing
            np_arr = np.fromstring(img.data, np.uint8)
            orig_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Flip upside down image
            orig_frame = cv2.flip(orig_frame, -1)

            self.get_target_points(orig_frame)
            # print('self.cam_features')
            # print(self.cam_features)

            # self.set_ideal_features()
            # print('self.ideal_features')
            # print(self.ideal_features)

            self.apply_vs()
            # print('self.vel')
            # print(self.vel)

            orig_frame = cv2.circle(orig_frame,(self.ideal_features[0],self.ideal_features[1]),radius = 10, color = (255,0,0), thickness = -1)
            orig_frame = cv2.circle(orig_frame,(self.ideal_features[2],self.ideal_features[3]),radius = 10, color = (255,0,0), thickness = -1)
            orig_frame = cv2.circle(orig_frame,(self.ideal_features[4],self.ideal_features[5]),radius = 10, color = (255,0,0), thickness = -1)

            self.show_image(orig_frame, "Quokka Servo 1")

        except Exception as e:
            print("Error in RGB image processing!")
            print(e)

    def get_target_points(self, orig_frame):
        """
        Defines a function to look for 3 coloured points  in a camera frame
        obtained by subscribing to the camera topic
        :param img: is the ROS RGB image from real sense camera
        """
        print("[INFO] get_target_points")
        try:
            # Apply a series of masks to the original image and obatin the
            # contours for each
            for i in range(len(self.masks)):
                # Apply the mask to the original image and find contours
                lower = np.array([self.masks[i][0, 0], self.masks[i][0, 1], self.masks[i][0, 2]])
                upper = np.array([self.masks[i][1, 0], self.masks[i][1, 1], self.masks[i][1, 2]])
                mask = cv2.inRange(orig_frame, lower, upper)
                # self.show_image(mask, "mask Image")
                contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                # Obtain the largest contour if contours have been found. Then find
                # the centroid of the contour and draw to the image
                if len(contours) > 0:
                    contour = sorted(contours, key=cv2.contourArea, reverse=True)[0]
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        Cx = int(M["m10"] / M["m00"])
                        Cy = int(M["m01"] / M["m00"])
                        cv2.circle(orig_frame, (Cx, Cy), 7, (255, 255, 255), -1)

                        self.cam_features[2*i] = Cx
                        self.cam_features[2*i+1] = Cy

            # self.show_image(orig_frame,"Original Image")

        except Exception as e:
            print("Error in get_target_points!")
            print(e)

    def set_ideal_features(self):
        print("[INFO] set_ideal_features")
        try:
            # Hard code 3 points on the screen to represent the desired distance from the robot (Measure this with the actual jenga set)
            if self.mode == "push":
                self.ideal_features = np.array([260, 360, 275, 340, 290, 360]).reshape((6,1))
            elif self.mode == "pull":
                self.ideal_features = np.array([260, 360, 275, 340, 290, 360]).reshape((6,1))
        except Exception as e:
            print("Error in set_ideal_features!")
            print(e)

    def apply_vs(self):

        print("[INFO] apply_vs")

        try:
            print("self.ideal_features")
            print(self.ideal_features)

            print("self.cam_features")
            print(self.cam_features)

            # Focal length of camera (fixed)
            f = int(self.K[0][0])

            for i in range(0, self.num_of_points):
                u = int(self.cam_features[i * 2][0])
                v = int(self.cam_features[i * 2 + 1][0])
                Z = float(self.depth_img[u, v])


                # self.jacobian_matrix[i * 2:i * 2 + 2, :] = \
                #     np.array([[-f / Z, 0, u / Z, u * v / f, -(f + u * u / f), v],
                #     [0, -f / Z, v / Z, (f + v * v / f), -u * v / f,-u]])

            # print("self.jacobian_matrix")
            # print(self.jacobian_matrix)

            # calculate error
            self.error = np.subtract(self.ideal_features, self.cam_features)
            print("error")
            print(self.error)

            x_error = -(self.error[0]+self.error[2]+self.error[4])/float(self.num_of_points)
            y_error = (self.error[1]+ self.error[3]+ self.error[5])/float(self.num_of_points)

            x_error = x_error * 0.0001* math.sqrt(abs(x_error))
            y_error = y_error * 0.0001 * math.sqrt(abs(y_error))

            print('x_error')
            print(x_error)
            print('y_error')
            print(y_error)


            # Publish the twist message
            twist_message = Twist()
            twist_message.linear.x = x_error
            twist_message.linear.y = y_error
            twist_message.linear.z = 0
            twist_message.angular.x = 0
            twist_message.angular.y = 0
            twist_message.angular.z = 0
            self.publisher.publish(twist_message)


            # # calculate velocity
            # self.vel = self.gain_lambda * np.dot(np.linalg.inv(self.jacobian_matrix), self.error)
            # print("command velocity")
            # print(self.vel)
            #
            # # Publish the twist message
            # twist_message = Twist()
            # twist_message.linear.x = self.vel[0][0]
            # twist_message.linear.y = self.vel[1][0]
            # twist_message.linear.z = self.vel[2][0]
            # twist_message.angular.x = self.vel[3][0]
            # twist_message.angular.y = self.vel[4][0]
            # twist_message.angular.z = self.vel[5][0]
            # self.publisher.publish(twist_message)

        except Exception as e:
            print("Error in creating image matrix")
            print(e)

    def cam_depth_callback(self, depth):
        """
        Function to obatin the depth image from the realsense
        :param depth: The depth input image from the realsense depth compressed
        image topic
        """
        print("[INFO] Depth recieved")
        try:
            # Read in the depth map and convert to an opencv type
            bridge = CvBridge()
            cvImg = bridge.imgmsg_to_cv2(depth, desired_encoding='16UC1')
            self.depth_img = cvImg

            # print(self.depth_img)

        except Exception as e:
            print("Error in depth processing!")
            print(e)

    def cam_info_callback(self, info):
        """
        Defines a function to obtain the camera instrinsics and
        other properties from the camera_info topic
        :param info: is the camera_info message
        """
        print("[INFO] Cam info recieved")
        temp = info.K
        self.K = np.array([[temp[0], temp[1], temp[2]], [temp[3], temp[4], temp[5]],[temp[6], temp[7], temp[8]]])


def main():
    """
    Main function for the landmark finder node
    """
    # Create a Visual Servoyer object
    vs = VisualServoyer()

    yellow = np.array([[20, 90, 80],
                     [50, 130, 130]])
    red = np.array([[0, 0, 80],
                    [20, 20, 110]])
    green = np.array([[0, 80, 0],
                      [20, 120, 20]])

    # Create a list of all the marker hsv colour ranges
    vs.masks = [yellow, red, green]
    vs.labels = ["yellow", "red", "green"]

    # Initialise the node and the publishers and subscribers
    rospy.init_node("visual_servoyer1", anonymous=True)

    # vs.publisher = rospy.Publisher("/landmarks", Landmarks_Msg, queue_size=1)
    vs.publisher = rospy.Publisher("/servoing_results1", Twist, queue_size=1)

    vs.cam_dep_sub = rospy.Subscriber("/realsense/depth/image_raw", Image_msg, vs.cam_depth_callback)
    vs.cam_colour_sub = rospy.Subscriber("/realsense/color/image_raw/compressed", CompressedImage,vs.camera_colour_callback)
    vs.cam_info_sub = rospy.Subscriber("/camera_info", CameraInfo, vs.cam_info_callback)

    # Prevent script execution completion and trigger the
    # ros clock spin
    rospy.spin()


# Main program entry point
if __name__ == "__main__":
    # Run the main program loop
    main()