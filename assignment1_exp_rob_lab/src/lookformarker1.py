#!/usr/bin/env python3

"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/color/image_raw (sensor_msgs.msg.Image)

Published Topics:
    /aruco_ID (std_msgs.msg.String)
       String containing the IDs of detected ArUco markers

Parameters:
    marker_size - size of the markers in meters (default 0.0625)
    aruco_dictionary_id - dictionary that was used to generate markers (default DICT_6X6_250)
    image_topic - image topic to subscribe to (default /camera/color/image_raw)

Author: Bernard Maacaron
Version: 10/26/2020
"""

import sys
import numpy as np
import cv2.aruco as aruco
import cv2
import roslib
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class MarkerDetector:

    def __init__(self):
        self.id_pub = rospy.Publisher("/aruco_ID", String, queue_size=1)
        self.id_sub = rospy.Subscriber("/aruco_ID", String, self.aruco_id_callback)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.detector_callback)
        self.bridge = CvBridge()
        self.robot_state = "searching"  # initialize the state of the robot, of course, at first, it is searching
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.ids_list =[]



    def detector_callback(self, raw_data):
        found_markers=0 #we cannot use ids_list because it can first detect marker 15, 12, 13 before the 11 and it will increase the size of the list 
        print(self.ids_list)
        while ((found_markers < 4) and (self.robot_state == "searching")):
            print("searching mode")
            self.move_robot(0.1, 0.1, rospy.Duration(10.0))  # MOVE and turn FORWARD SLOWLY FOR 10 SECONDS
            if self.id_sub.get_num_connections() > 0:  # Check if there are subscribers to /aruco_ID topic meaning a marker is detected 
                if "11" in self.ids_list:
                    print("marker 11 found!, searching for 12")
                    if found_markers <1: found_markers +=1
                    print('the number of found markers is,'+ str(found_markers))
                    #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    #YOU SHOULD CHANGE THE MOVEMENT OF THE ROBOT SO THAT IT GETS CLOSER TO MARKER 12
                    #SO THAT THE ROBOT DOES NOT SPEND MUCH TIME TURNING LOOKING FOR THE NEXT MARKER 
                    self.move_robot(-0.1, 0.0, rospy.Duration(5.0))  # MOVE BACKWARD FOR 5 SECONDS
                    self.move_robot(0.1, 0.0, rospy.Duration(5.0))  # TURN SLOWLY FOR 5 SECONDS
                    if "12" in self.ids_list:
                            print("marker 12 found!, searching for 13")
                            if found_markers < 2: found_markers +=1
                            print('the number of found markers is,'+ str(found_markers))
                            #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                            #YOU SHOULD CHANGE THE MOVEMENT OF THE ROBOT SO THAT IT GETS CLOSER TO MARKER 13
                            #SO THAT THE ROBOT DOES NOT SPEND MUCH TIME TURNING LOOKING FOR THE NEXT MARKER 
                            self.move_robot(-0.1, 0.0, rospy.Duration(5.0))  # MOVE BACKWARD FOR 5 SECONDS
                            self.move_robot(0.1, 0.0, rospy.Duration(5.0))  # TURN SLOWLY FOR 5 SECONDS
                            if "13" in self.ids_list:
                                print("marker 13 found!, searching for 15")
                                if found_markers <3: found_markers +=1
                                print('the number of found markers is,'+ str(found_markers))
                                #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                #YOU SHOULD CHANGE THE MOVEMENT OF THE ROBOT SO THAT IT GETS CLOSER TO MARKER 15
                                #SO THAT THE ROBOT DOES NOT SPEND MUCH TIME TURNING LOOKING FOR THE NEXT MARKER 
                                self.move_robot(-0.1, 0.0, rospy.Duration(5.0))  # MOVE BACKWARD FOR 5 SECONDS
                                self.move_robot(0, 0.1, rospy.Duration(5.0))  # TURN SLOWLY FOR 5 SECONDS
                                if "15" in self.ids_list:
                                    print("marker 15 found!")
                                    print("DOOOOOOOOONE!")
                                    if found_markers < 4: found_markers +=1     
                                    print('the number of found markers is,'+ str(found_markers))
                                    self.robot_state = "stop"  # stop searching
                                    self.move_robot(1, 0.0, rospy.Duration(5.0))  # STOP ROBOT
                                    break
                                
                            
                    
                    
                
            
        

    # subscribing to aruco_id callback
    def aruco_id_callback(self, msg):
        received_id = msg.data
        if received_id not in self.ids_list:
            self.ids_list.append(received_id)
        else:
            print(f"The marker {received_id} was already found")

        print("The IDs list is now:", self.ids_list)
        


    def ImageConverter_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        return cv_image

    

    #def detect_aruco(self,cv_image):
     #   gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) # Decide if this is needed
     #   aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
     #   parameters = aruco.DetectorParameters()
     #  corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
     #  output = aruco.drawDetectedMarkers(cv_image, corners, ids)  # detect the sruco markers and display its aruco id.
     #  return output,ids
    


    #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    #BLOCK TO FOLLOW THE ARUCO MARKER ONCE DETECTED, the corners are from detect_aruco
    #def move_towards_arucomarker(self,corners, target_x, target_y):
        # Calculate the center of the marker
        #center_x = (corners
        #center_y = (corners

        # Calculate the difference between the current position and the target position
        #delta_x = target_x - current_x
        #delta_y = target_y - current_y

        # move the robot towards the center of the markers 
        #self.move_robot(delta_x, delta_y, rospy.Duration(5.0))  



    def move_robot(self, linear_speed, angular_speed, duration):
        move_cmd = Twist()
        move_cmd.linear.x = linear_speed
        move_cmd.angular.z = angular_speed

        stop_cmd = Twist()

        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < duration:
            self.cmd_vel_pub.publish(move_cmd)
            rospy.sleep(0.1)

        self.cmd_vel_pub.publish(stop_cmd)
        rospy.sleep(1)



def main(args):
    print("lookformarker node start")
    rospy.init_node('lookformarker')
    marker_detector = MarkerDetector()
    rospy.spin()

if __name__ == "__main__":
    try:
        main(sys.argv)
    # exception problems handled here kill node
    except rospy.ROSInterruptException:
        pass
