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
"""
        
import cv2.aruco as aruco
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError


class MarkerDetector:
    # ==================== INITIALIZATION ====================
    def __init__(self):
        # Initialize variables
        self.ids_list = None
        self.bridge = CvBridge()

        # Image related topics
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.detector_callback)
        
        self.markerData_pub = rospy.Publisher("/aruco_Marker_Data", String, queue_size=1)
        self.image_pub = rospy.Publisher("/aruco_Image", Image, queue_size=1)

        
    # ==================== MARKER DETECTION ====================
    def detector_callback(self, raw_data):
        cv_image = self.ImageConverter_callback(raw_data)
        self.markers_img, self.ids_list, self.corners_list = self.detect_aruco(cv_image)

        if self.markers_img is not None:
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.markers_img, "bgr8"))
            except CvBridgeError as e:
                print(e)

        if self.ids_list is None:
            self.markerData_pub.publish("No markers found.")
        else:
            for i in range(len(self.ids_list)):
                # Access corner coordinates of the i-th marker
                marker_corners = self.corners_list[i]

                # Create a custom message with corner coordinates
                message = f"Marker {self.ids_list[i]} Corners: {marker_corners}"

                # Publishing the message
                self.markerData_pub.publish(message)

    # Used to convert the ROS image to OpenCV format - called in detector_callback
    def ImageConverter_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        return cv_image
        
    # Used to detect the ArUco markers - called in detector_callback 
    def detect_aruco(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        corners_list, ids_list, _ = aruco.detectMarkers(gray, aruco_dict)
        output = aruco.drawDetectedMarkers(cv_image, corners_list, ids_list)
        return output, ids_list, corners_list



def main():
    print("Initializing ROS Node - lookformarker")
    rospy.init_node('lookformarker')
    marker_detector = MarkerDetector()
    try:
        rospy.spin()
    except:
        print("Shutting down ROS Node - lookformarker")
    cv2.destroyAllWindows()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

