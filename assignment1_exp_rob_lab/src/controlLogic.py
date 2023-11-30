import numpy as np
import ast  # Using ast.literal_eval for safe string to list conversion

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Marker IDs to detect, their order and their corresponding detection status (True/False)
markerDetected_dict = {11:False, 12:False, 13:False, 15:False}

# Robot states dictionary
robotState_dict = {1:"Searching", 2:"Moving", 3:"Finished"}

# Minimum required pixel size
requiredEdgeSize = 200

# Camera Resolution
camera_resolution = (640, 480)


class controlLogic:
    # ==================== INITIALIZATION ====================
    def __init__(self):
        # Initialize variables
        self.robotState = robotState_dict[1] #initialize the state of the robot of course at first it is searching
        self.ids_list = None

        # Image related topics
        self.markerData_sub = rospy.Subscriber("/aruco_Marker_Data", String, self.runControlLogic)


        # Movement related topics
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    #==================== ROBOT MOVEMENT ====================
    def runControlLogic(self,raw_markerData):
        # for marker, detectStat in markerDetected_dict.items():
        #     while detectStat == False and self.robotState != "Finished":
        
        markerID, markerCorners = self.markerDataConverter(raw_markerData)
        print("Robot State:", self.robotState)
        print("Marker ID:", markerID)
        print("Marker Corners:", markerCorners)
        
        marker = [key for key,values in markerDetected_dict.items() if values == False][0]
        print("Marker:", marker)
        if self.robotState == "Searching":
            while markerID == None:
                self.move_robot(0.0, 0.4)
                print("THE ROBOT SHOULD ROTATE NOW")

            if marker in self.ids_list:
                print("Correct Marker detected:", marker)
                self.move_robot(0.0, 0.0)
                self.robotState = robotState_dict[2]
                
        while self.robotState == "Moving":
            #DONE -  Extract desired marker corners
            index = self.ids_list.index(marker)
            corners = self.corner_list[index]
            
            #TODO: move towards the desired marker center until the marker edge size is confirmed
            self.move_towards_marker(corners)
            while True:
                if self.calculateEdgeSize(corners) >= requiredEdgeSize:
                    self.move_robot(0.0, 0.0)
                    markerDetected_dict[marker] = True
                    break
                else:
                    print("Moving towards marker:", marker)
        # else:
        #     print("Invalid state")
        #     rospy.loginfo("Invalid state")

        self.robotState = robotState_dict[3]
                       
    def move_towards_marker(self, corners):
        # Calculate the center of the marker
        center_x = np.mean([corner[0] for corner in corners])
        center_y = np.mean([corner[1] for corner in corners])

        # Get the center of the camera frame
        camera_center = camera_resolution/2  # Adjust this based on your camera resolution

        # Calculate the movement required
        delta_x = center_x - camera_center[0]

        # Move the robot proportionally to the distance from the center
        linear_speed = 0.1  # Adjust as needed
        angular_speed = 0.1  # Adjust as needed

        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = -angular_speed * delta_x / camera_center[0]

        # Publish the velocity command
        self.cmd_vel_pub.publish(twist)
              
    def calculateEdgeSize(self, corners):
        # Calculate Edge size
        edge_lengths = [np.linalg.norm(np.array(corners[i + 1]) - np.array(corners[i])) for i in range(3)]
        marker_size = max(edge_lengths)
        return marker_size

    def move_robot(self, linear_speed, angular_speed):
        move_cmd = Twist()
        move_cmd.linear.x = linear_speed
        move_cmd.angular.z = angular_speed
        self.cmd_vel_pub.publish(move_cmd)
        rospy.sleep(1)      

    def markerDataConverter(self, raw_markerData):
        raw_markerData = raw_markerData.data
        
        if raw_markerData == "No markers found.":
            return None, None
        else:
            # Extract marker ID
            marker_id = int(raw_markerData.split()[1])

            # Extract corner coordinates using ast.literal_eval
            raw_corners = raw_markerData.split(":")[1].strip()
            corner_coordinates = ast.literal_eval(raw_corners)

            # Print or use the extracted information
            print(f"Marker ID: {marker_id}")
            print(f"Corner Coordinates: {corner_coordinates}")
            
            return marker_id, corner_coordinates


def main():
    print("Initializing ROS Node - controlLogic")
    rospy.init_node('controlLogic')
    controller = controlLogic()
    try:
        rospy.spin()
    except:
        print("Shutting down ROS Node - controlLogic")



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

