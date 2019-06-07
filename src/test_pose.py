#!/usr/bin/env python
# -*- coding: utf-8 -*-



#from __future__ import print_function

import roslib

import sys
import rospy
import datetime
import numpy as np
import string
import requests
import json
import rospy

import base64
import math
import time
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, PointCloud
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image, CameraInfo
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

in_process = False


def AngleProxy(A1=0, A2=0):
    A1 = A2 - A1
    A1 = (A1 + math.pi) % (2 * math.pi) - math.pi
    return A1


def getAngle(a, b, c):
    if a[0] == 0 or b[0] == 0 or c[0] == 0 or a[1] == 0 or b[1] == 0 or c[1] == 0:
        return 999
    else:
        return abs(
            int(math.degrees(AngleProxy(math.atan2(c[1] - b[1], c[0] - b[0]), math.atan2(a[1] - b[1], a[0] - b[0])))))


class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("/pose_detection/image", Image, queue_size=100)
        self.pointcloud_pub = rospy.Publisher("/pose_detection/pointclouds", PointCloud, queue_size=100)
        in_process = False
        self.bridge = CvBridge()
        self.object = cv2.imread("sims.png", cv2.IMREAD_UNCHANGED)
        self.eye = cv2.imread("eye.png", cv2.IMREAD_UNCHANGED)
        self.mouth = cv2.imread("mouth.png", cv2.IMREAD_UNCHANGED)

        self._MIN_DIST = rospy.get_param("minimum_distance", 0.2)
        self._MAX_DIST = rospy.get_param("maximum_distance", 50.0)
        self._CAM_ANGLE_WIDTH = rospy.get_param("camera_angle_width", 1.012290966)
        self._CAM_ANGLE_HEIGHT = rospy.get_param("camera_angle_height", 0.785398163397)



        self.image_sub = message_filters.Subscriber("camera/rgb/image_raw", Image, queue_size=100)
        self.depth_sub = message_filters.Subscriber("camera/depth/image_raw", Image, queue_size=100)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 100, 0.1)
        self.ts.registerCallback(self.callback)
        rospy.loginfo("pifpaf ready")

    def callback(self, rgb_data, depth_data):
        if rospy.Time.now() - rgb_data.header.stamp < rospy.Duration(0.1):
            global in_process
            if not in_process:
                in_process = True
                rospy.loginfo("synced images!")

                try:
                    depth_image = self.bridge.imgmsg_to_cv2(depth_data, "passthrough")
                except CvBridgeError:
                    print(CvBridgeError)

                depth_array = np.array(depth_image, dtype=np.float32)

                time1 = time.time()
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
                except CvBridgeError as e:
                    print(e)

                filename = "cam_image.jpeg"
                score = 0.3

                # cam = cv2.VideoCapture(0)
                # cv2.namedWindow("camera_w_pose")

                # while True:
                # ret, frame = cam.read()
                # cv2.imwrite(filename, cv_image)

                retval, buffer = cv2.imencode('.jpg', cv_image)
                # image_file = open(filename, "rb")
                # encoded_string = base64.b64encode(image_file.read())
                encoded_string = base64.b64encode(buffer)

                headers = {
                    'Content-Type': 'application/json',
                }

                json_data = '{"image": "data:image/jpeg;base64,' + str(encoded_string.decode("utf-8")) + '"}'
                File_object = open("/tmp/test.json", "w+")
                File_object.write(json_data)
                File_object.close()

                data = open("/tmp/test.json")
                # data_str =
                response = requests.post('http://localhost:5000/process', headers=headers, data=str(json_data))

                # print(response.content[0])
                json_content = json.loads(response.content)
                time2 = time.time()
                # rospy.loginfo(time2-time1)
                # for i in json_content[0]['coordinates']:
                # print

                # image = cv2.imread(filename)

                # print(response.content)
                height, width, channels = cv_image.shape
                font = cv2.FONT_HERSHEY_SIMPLEX
                person_it = 0

                # get pixel to rad ratio
                xratio = self._CAM_ANGLE_WIDTH / width;
                yratio = self._CAM_ANGLE_HEIGHT / height;


                #   it = 0
                #   if j['score'] > score:
                #     person_it = person_it+1
                #     cv2.putText(cv_image,"Person " + str(person_it),(int(j['coordinates'][0][0]*width-60),int(j['coordinates'][0][1]*height-60)), font, 1,(0,0,255),1,cv2.LINE_AA)
                #     for i in j['coordinates']:
                #       it = it+1
                #       #print(i)
                #       cv2.circle(cv_image,(int(i[0]*width),int(i[1]*height)), 4, (0,0,255), -1)
                #       cv2.putText(cv_image,str(it),(int(i[0]*width),int(i[1]*height)), font, 0.4,(0,255,0),1,cv2.LINE_AA)
                rospy.loginfo("***************************************")
                for j in json_content:
                    it = 0
                    if j['score'] > score:




                        pointList = []
                        rospy.loginfo("good score")
                        for point in j['coordinates']:

                            try:
                                if point[0] != 0 and point[1] != 0:
                                    pixel_x = min(width-1, max(0, int(point[0] * width)))
                                    pixel_y= min(height-1, max(0, int(point[1] * height)))
                                    pixel_depth = np.mean(depth_array[pixel_y-3:pixel_y+3, pixel_x-3:pixel_x+3])/1000

                                    if pixel_depth != 0:
                                        # get the IRL angles from the camera center to the point double
                                        ax = -(pixel_x-width/2)*xratio # pixel to angle
                                        ay = -(pixel_y-height/2)*yratio # pixel to angle

                                        # Convert the angeles and distance to x y z coordinates
                                        point = Point32()
                                        point.z = pixel_depth * math.cos(ax) * math.cos(ay)  # ang to 3D point (rad to m)
                                        point.x = -pixel_depth * math.sin(ax)  # ang to 3D point (rad to m)
                                        point.y = -pixel_depth * math.sin(ay)  # ang to 3D point (rad to m)

                                        pointList.append(point)

                            except:
                                rospy.logerr("Couldn't get point in z space")

                        pointcloud = PointCloud()
                        pointcloud.header = rgb_data.header
                        if len(pointList):
                            # Calculate the center
                            meanX = 0
                            meanY = 0
                            meanZ = 0
                            for point in pointList:
                                meanX += point.x
                                meanY += point.y
                                meanZ += point.z
                            meanX /= len(pointList)
                            meanY /= len(pointList)
                            meanZ /= len(pointList)

                            # Eliminate extreme points
                            for point in pointList:
                                if abs(meanZ-point.z)**2 < 0.2:
                                    pointcloud.points.append(point)


                        self.pointcloud_pub.publish(pointcloud)

                        person_it = person_it + 1
                        # cv2.putText(cv_image,"Person " + str(person_it),(int(j['coordinates'][0][0]*width-60),int(j['coordinates'][0][1]*height-60)), font, 1,(0,0,255),1,cv2.LINE_AA)

                        # Nombre de points : 17 [0-16]
                        # 0 : nez
                        # 1 : oeil gauche
                        # 2 : oeil droit
                        # 3 : oreille gauche
                        # 4 : oreille droite
                        # 5 : epaule gauche
                        # 6 : epaule droite
                        # 7 : coude gauche
                        # 8 : coude droit
                        # 9 : main gauche
                        # 10 : main droite
                        # 11 : hanche gauche
                        # 12 : hanche droite
                        # 13 : genoux gauche
                        # 14 : genoux droit
                        # 15 : pied gauche
                        # 16 : pied droit

                        # [0] = X
                        # [1] = Y

                        # print(i)
                        rospy.loginfo("-------------------------------")
                        rospy.loginfo("Personne " + str(person_it))
                        if j['coordinates'][9][1] < j['coordinates'][0][1] and j['coordinates'][9][1] > 0:
                            cv2.putText(cv_image, "Bras gauche leve", (
                            int(j['coordinates'][9][0] * width - 10), int(j['coordinates'][9][1] * height - 10)), font,
                                        0.45, (0, 255, 0), 1, cv2.LINE_AA)
                            rospy.loginfo("Bras gauche leve")
                        if j['coordinates'][10][1] < j['coordinates'][0][1] and j['coordinates'][10][1] > 0:
                            cv2.putText(cv_image, "Bras droit leve", (
                            int(j['coordinates'][10][0] * width - 10), int(j['coordinates'][10][1] * height - 10)), font,
                                        0.45, (0, 255, 0), 1, cv2.LINE_AA)
                            rospy.loginfo("Bras droit leve")

                        # for i in j['coordinates']:
                        #  cv2.circle(cv_image,(int(i[0]*width),int(i[1]*height)), 4, (0,0,255), -1)

                        if j['coordinates'][5][1] > 0 and j['coordinates'][7][1] > 0:
                            cv2.line(cv_image, (int(j['coordinates'][5][0] * width), int(j['coordinates'][5][1] * height)),
                                     (int(j['coordinates'][7][0] * width), int(j['coordinates'][7][1] * height)),
                                     (0, 0, 255), 5)
                        if j['coordinates'][5][1] > 0 and j['coordinates'][6][1] > 0:
                            cv2.line(cv_image, (int(j['coordinates'][5][0] * width), int(j['coordinates'][5][1] * height)),
                                     (int(j['coordinates'][6][0] * width), int(j['coordinates'][6][1] * height)),
                                     (255, 0, 190), 5)

                        if j['coordinates'][6][1] > 0 and j['coordinates'][8][1] > 0:
                            cv2.line(cv_image, (int(j['coordinates'][6][0] * width), int(j['coordinates'][6][1] * height)),
                                     (int(j['coordinates'][8][0] * width), int(j['coordinates'][8][1] * height)),
                                     (255, 0, 0), 5)
                        if j['coordinates'][7][1] > 0 and j['coordinates'][9][1] > 0:
                            cv2.line(cv_image, (int(j['coordinates'][7][0] * width), int(j['coordinates'][7][1] * height)),
                                     (int(j['coordinates'][9][0] * width), int(j['coordinates'][9][1] * height)),
                                     (0, 0, 255), 5)
                        if j['coordinates'][8][1] > 0 and j['coordinates'][10][1] > 0:
                            cv2.line(cv_image, (int(j['coordinates'][8][0] * width), int(j['coordinates'][8][1] * height)),
                                     (int(j['coordinates'][10][0] * width), int(j['coordinates'][10][1] * height)),
                                     (255, 0, 0), 5)
                        if j['coordinates'][11][1] > 0 and j['coordinates'][13][1] > 0:
                            cv2.line(cv_image,
                                     (int(j['coordinates'][11][0] * width), int(j['coordinates'][11][1] * height)),
                                     (int(j['coordinates'][13][0] * width), int(j['coordinates'][13][1] * height)),
                                     (0, 0, 255), 5)
                        if j['coordinates'][12][1] > 0 and j['coordinates'][14][1] > 0:
                            cv2.line(cv_image,
                                     (int(j['coordinates'][12][0] * width), int(j['coordinates'][12][1] * height)),
                                     (int(j['coordinates'][14][0] * width), int(j['coordinates'][14][1] * height)),
                                     (255, 0, 0), 5)
                        if j['coordinates'][13][1] > 0 and j['coordinates'][15][1] > 0:
                            cv2.line(cv_image,
                                     (int(j['coordinates'][13][0] * width), int(j['coordinates'][13][1] * height)),
                                     (int(j['coordinates'][15][0] * width), int(j['coordinates'][15][1] * height)),
                                     (0, 0, 255), 5)
                        if j['coordinates'][14][1] > 0 and j['coordinates'][16][1] > 0:
                            cv2.line(cv_image,
                                     (int(j['coordinates'][14][0] * width), int(j['coordinates'][14][1] * height)),
                                     (int(j['coordinates'][16][0] * width), int(j['coordinates'][16][1] * height)),
                                     (255, 0, 0), 5)
                        if j['coordinates'][11][1] > 0 and j['coordinates'][12][1] > 0:
                            cv2.line(cv_image,
                                     (int(j['coordinates'][11][0] * width), int(j['coordinates'][11][1] * height)),
                                     (int(j['coordinates'][12][0] * width), int(j['coordinates'][12][1] * height)),
                                     (255, 0, 190), 5)

                        # ANGLE BRAS 180+-10 : Pointe
                        # if getAngle((j['coordinates'][5][1], j['coordinates'][5][0]), (j['coordinates'][7][1], j['coordinates'][7][0]), (j['coordinates'][9][1], j['coordinates'][9][0])):

                        angleBrasGauche = getAngle((j['coordinates'][5][1], j['coordinates'][5][0]),
                                                   (j['coordinates'][7][1], j['coordinates'][7][0]),
                                                   (j['coordinates'][9][1], j['coordinates'][9][0]))
                        angleBrasDroit = getAngle((j['coordinates'][6][1], j['coordinates'][6][0]),
                                                  (j['coordinates'][8][1], j['coordinates'][8][0]),
                                                  (j['coordinates'][10][1], j['coordinates'][10][0]))
                        angleEpauleGauche = getAngle((j['coordinates'][11][1], j['coordinates'][11][0]),
                                                     (j['coordinates'][5][1], j['coordinates'][5][0]),
                                                     (j['coordinates'][7][1], j['coordinates'][7][0]))
                        angleEpauleDroit = getAngle((j['coordinates'][12][1], j['coordinates'][12][0]),
                                                    (j['coordinates'][6][1], j['coordinates'][6][0]),
                                                    (j['coordinates'][8][1], j['coordinates'][8][0]))

                        if not angleEpauleGauche == 999:
                            cv2.putText(cv_image, str(angleEpauleGauche),
                                        (int(j['coordinates'][5][0] * width - 10), int(j['coordinates'][5][1] * height)),
                                        font, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
                        if not angleEpauleDroit == 999:
                            cv2.putText(cv_image, str(angleEpauleDroit),
                                        (int(j['coordinates'][6][0] * width - 10), int(j['coordinates'][6][1] * height)),
                                        font, 0.45, (0, 255, 0), 1, cv2.LINE_AA)

                        if not angleBrasGauche == 999:
                            if angleBrasGauche > 120 and angleEpauleGauche > 20:
                                cv2.putText(cv_image, str("Bras gauche pointe"),
                                            (int(j['coordinates'][9][0] * width), int(j['coordinates'][9][1] * height)),
                                            font, 0.45, (0, 255, 0), 1, cv2.LINE_AA)

                            cv2.putText(cv_image, str(angleBrasGauche),
                                        (int(j['coordinates'][7][0] * width - 10), int(j['coordinates'][7][1] * height)),
                                        font, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
                        if not angleBrasDroit == 999:
                            if angleBrasDroit > 120 and angleEpauleDroit > 20:
                                cv2.putText(cv_image, str("Bras droit pointe"),
                                            (int(j['coordinates'][10][0] * width), int(j['coordinates'][10][1] * height)),
                                            font, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
                            cv2.putText(cv_image, str(angleBrasDroit),
                                        (int(j['coordinates'][8][0] * width - 10), int(j['coordinates'][8][1] * height)),
                                        font, 0.45, (0, 255, 0), 1, cv2.LINE_AA)

                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                except CvBridgeError as e:
                    print(e)

                in_process = False


            else:
                rospy.loginfo("skipping frame")
        else:
                rospy.loginfo("too late")

        # DEBOUT / ASSIS / COUCHE
        # Debout :

        # cv2.putText(cv_image,str(it),(int(i[0]*width),int(i[1]*height)), font, 0.4,(0,255,0),1,cv2.LINE_AA)
        # cv2.imshow('camera_w_pose',image)

        # cv2.waitKey(1)

        # rospy.init_node('image_converter', anonymous=True)
        # ic = image_converter()

        # try:
        #  rospy.spin()
        # except KeyboardInterrupt:
        #  print("Shutting down")
        # cv2.destroyAllWindows()



# rospy.loginfo(time.time() - time1)

def main(args):
    rospy.init_node('image_converter')
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
