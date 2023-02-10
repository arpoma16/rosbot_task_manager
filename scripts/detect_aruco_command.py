#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
from cv_bridge import CvBridge
import cv2
import numpy as np
from rosbot_task_manager.srv import Command_service
import get_yalm 



class Nodo(object):
    def __init__(self):
        # Params
        self.aruco_img = None
        self.aruco_id = 0
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub_img = rospy.Publisher('aruco/image', Image,queue_size=10)
        self.pub_id = rospy.Publisher('aruco/id', UInt8,queue_size=10)

        # Subscribers
        rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

        # list of commands
        path_yalm = rospy.get_param('~yalm_path')
        self.list_command =  get_yalm.get_command(path_yalm)


    def callback(self, msg):
        rospy.loginfo('Image received...')
        self.aruco_id = 0
        image = self.br.imgmsg_to_cv2(msg)
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,parameters=arucoParams)
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                # draw the bounding box of the ArUCo detection
                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                print("[INFO] ArUco marker ID: {}".format(markerID))
                self.aruco_id = markerID
                # show the output image

        for c in range(0,self.list_command['command_num']) :
            if c == self.aruco_id -1:
                command = self.list_command['command']['command'+str(c)]
                print("comando reconocido: " + command['name'])
                self.Send_command(command["name"])
                break

        self.aruco_img = image


    def start(self):
        rospy.loginfo("Timing images")
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            if self.aruco_img is not None:
                self.pub_img.publish(self.br.cv2_to_imgmsg(self.aruco_img))
            self.pub_id.publish(self.aruco_id)
            self.loop_rate.sleep()

    def Send_command(self,command_recognice):
        rospy.wait_for_service('Service_command')
        try:
            send_commad_srv = rospy.ServiceProxy('Service_command', Command_service)
            resp1 = send_commad_srv(command_recognice)
            return resp1.c
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

if __name__ == '__main__':
    rospy.init_node("Aruco_basic_detector", anonymous=True)
    my_node = Nodo()
    my_node.start()