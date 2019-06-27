import socket
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
import rospy
from sensor_msgs.msg import Image

from common import send

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(SERVER)

bridge = CvBridge()

dataInfo = {
  'id': 'video0',
  'streamType': 'video',
  'dataType': 'jpg',
  'sequence': 1,
  'timestamp': 1
}

def onData(imgMsg):
  img = bridge.imgmsg_to_cv2(imgMsg, 'bgr8')
  jpgImg = cv.imencode('.jpg', frame)[1]
  data = np.array(jpgImg)
  
  send(s, info, data, 'array')
  
  info['sequence'] += 1
  info['timestamp'] += 1

if __name__ == "__main__":
  
  info = {
    'id': 'video0',
    'streamType': 'meta',
    'dataType': 'json',
    'sequence': 122,
    'timestamp': 0
  }
  dataOrigin = {
    'width': 640,
    'height': 480
  }
  data = json.dumps(dataOrigin)
  send(s, info, data, 'string')

  rospy.init_node('vizVideoSender', anonymous=True)
  rospy.Subscriber('usb_cam_image_raw', Image, onData)
  rospy.spin()

  s.close()
