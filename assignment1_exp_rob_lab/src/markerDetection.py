#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from aruco_ros import aruco_ros_utils

def image_callback(msg):
    try:
        # Convert ROS Image message to OpenCV image
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Create an ArUco marker detector
        marker_detector = aruco_ros_utils.MarkerDetector()
        
        # Detect markers in the image
        markers = marker_detector.detect(image)

        # Print marker information to console
        for marker in markers:
            print(marker)
            
            # Draw the detected markers in the image
            marker.draw(image, (0, 0, 255), 2)

        # Display the image with detected markers
        cv2.imshow("image", image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", e)

def main():
    rospy.init_node("aruco_marker_detection")
    image_transport = cv2.createImageTransport("/camera/image_raw")
    
    # Define the image subscriber to receive images from a ROS topic
    image_sub = image_transport.subscribe("/camera/image_raw", Image, image_callback)

    rospy.spin()

if __name__ == "__main__":
    main()
