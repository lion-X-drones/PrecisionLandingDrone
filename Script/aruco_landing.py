#!/usr/bin/env python
import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
from std_msgs.msg import String
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point
from sensor_msgs.msg import Image
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def wait_for(predicate, timeout):
    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now()
    while not predicate() and rospy.Time.now() - start_time < rospy.Duration(timeout):
        rate.sleep()
    return rospy.Time.now() - start_time < rospy.Duration(timeout)

def arm_takeoff():
    state_sub = rospy.Subscriber('/mavros/state', State, state_cb)
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

    rospy.loginfo("Waiting for MAVROS connection...")
    wait_for(lambda: current_state.connected, 30.0)
    rospy.loginfo("MAVROS connected")

    rate = rospy.Rate(20.0)  # 20 Hz

    # Change mode to GUIDED
    rospy.loginfo("Changing mode to GUIDED")
    if not wait_for(lambda: current_state.mode == "GUIDED", 5.0):
        set_mode_client(custom_mode="GUIDED")

    # Arm the vehicle
    rospy.loginfo("Arming")
    if not wait_for(lambda: current_state.armed, 2.0):
        arming_client(True)
    
    for i in range(1, 70):
        rate.sleep()

    # Takeoff
    rospy.loginfo("Taking off")
    if takeoff_client(altitude=4, latitude=0, longitude=0, min_pitch=0, yaw=0):
        rospy.loginfo("Takeoff sent")
    else:
        rospy.loginfo("Failed to send takeoff command")
    for i in range(1, 70):
        rate.sleep()
    

# Define multiple ArUco dictionaries
aruco_dicts = [
    cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250),
    cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_250),
    cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250),
    cv.aruco.getPredefinedDictionary(cv.aruco.DICT_7X7_250)
]

# Initialize the detector parameters
parameters = cv.aruco.DetectorParameters()
parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 23
parameters.adaptiveThreshWinSizeStep = 10
parameters.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
parameters.cornerRefinementWinSize = 5
parameters.cornerRefinementMaxIterations = 30
parameters.cornerRefinementMinAccuracy = 0.1

class ArUcoDetectorNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.vel_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        # Adjust the camera index as needed
        # self.cap = cv.VideoCapture(1)
        self.X = 2
        self.Y = 2
        self.Z = 4
        local = PoseStamped()
        rate = rospy.Rate(50)

        for i in range(50):
            local.pose.position.x = 2
            local.pose.position.y = 2
            local.pose.position.z = 4
            self.vel_pub.publish(local)
            rate.sleep()

    def image_callback(self):
        
        
        cap = cv.VideoCapture(1)
        n=1
        for i in range(n):
            _,cv_image = cap.read()
            self.detect_and_draw_markers(cv_image)

    def detect_and_draw_markers(self, frame):

        height, width, _ = frame.shape
        vel_msg=Point()
        # Draw a small square at the center of the frame
        square_size = 125  # Size of the square
        center_x, center_y = width // 2, height // 2
        top_left = (center_x - square_size // 2, center_y - square_size // 2)
        bottom_right = (center_x + square_size // 2, center_y + square_size // 2)

        for dictionary in aruco_dicts:
            # Detect markers
            markerCorners, markerIds, _ = cv.aruco.detectMarkers(frame, dictionary, parameters=parameters)

            if markerIds is not None:
                # Draw the bounding box around the detected ArUco markers
                for corners in markerCorners:
                    corners = corners.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners

                    # Convert each point to integers
                    topLeft = (int(topLeft[0]), int(topLeft[1]))
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))

                    # Calculate the center of the marker
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                    # Check if the center is in the free space
                    if top_left[0] <= cX <= bottom_right[0] and top_left[1] <= cY <= bottom_right[1]:
                        self.Z -= 0.2
                    else:
                        # Determine which quadrant the center is in
                        if cX < center_x and cY < center_y:
                            self.X += 0.15
                            self.Y -= 0.15
                        elif cX >= center_x and cY < center_y:
                            self.X -= 0.15
                            self.Y -= 0.15
                        elif cX < center_x and cY >= center_y:
                            self.X += 0.15
                            self.Y += 0.15
                        else:
                            self.X -= 0.15
                            self.Y += 0.15

        local = PoseStamped()
        rate = rospy.Rate(50)

        for i in range(50):
            local.pose.position.x = self.X
            local.pose.position.y = self.Y
            local.pose.position.z = self.Z
            self.vel_pub.publish(local)
            rate.sleep()
        
        rospy.loginfo(f"[{self.X} {self.Y} {self.Z}]")

    def run(self):
        self.image_callback()

if __name__ == '__main__':
    rospy.init_node("human_node", anonymous=True)
    arm_takeoff()
    node = ArUcoDetectorNode()
    rate=rospy.Rate(1)
    for i in range(1,5):
        rate.sleep()
    while not rospy.is_shutdown():
        node.run()
        # rate.sleep() 
