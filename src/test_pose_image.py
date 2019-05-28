#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('wm_dataset_preparation')
import sys
import rospy

import numpy as np
import string
import requests

from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from wm_dataset_preparation.cfg import object_extractionConfig

def main(args):
  import requests
  import json 
  import cv2 
  import base64
  
  filename = "test2.jpg"
  score = 0.0
  
  #cam = cv2.VideoCapture(0)
  cv2.namedWindow("camera_w_pose")

  #cv2.imwrite("cam_image.jpeg", frame)


  image_file = open(filename, "rb")
  encoded_string = base64.b64encode(image_file.read())
  


  headers = {
      'Content-Type': 'application/json',
  }

  json_data = '{"image": "data:image/jpeg;base64,'+str(encoded_string.decode("utf-8"))+'"}'
  File_object = open("test.json","w+")
  File_object.write(json_data)
  File_object.close()

  data = open('test.json')
  #data_str = 
  response = requests.post('http://localhost:5000/process', headers=headers, data=json_data)
  
  #print(response.content[0])
  json_content = json.loads(response.content)

  #for i in json_content[0]['coordinates']:
  # print
  
  
  image = cv2.imread(filename)
  
  #print(response.content)
  height, width, channels = image.shape
  font = cv2.FONT_HERSHEY_SIMPLEX
  person_it = 0
  for j in json_content:
    
    it = 0
    if j['score'] > score:
      person_it = person_it+1
      cv2.putText(image,"Person " + str(person_it),(int(j['coordinates'][0][0]*width-60),int(j['coordinates'][0][1]*height-60)), font, 1,(0,0,255),1,cv2.LINE_AA)
      for i in j['coordinates']:
        it = it+1
        #print(i)
        cv2.circle(image,(int(i[0]*width),int(i[1]*height)), 4, (0,0,255), -1)
        cv2.putText(image,str(it),(int(i[0]*width),int(i[1]*height)), font, 0.4,(0,255,0),1,cv2.LINE_AA)

  cv2.imshow('camera_w_pose',image)
  
  cv2.waitKey(0)
  
  #rospy.init_node('image_converter', anonymous=True)
  #ic = image_converter()
  
  #try:
  #  rospy.spin()
  #except KeyboardInterrupt:
  #  print("Shutting down")
  #cv2.destroyAllWindows()
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)