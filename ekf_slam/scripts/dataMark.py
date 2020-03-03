import socket
import json
import cv2 as cv
import numpy as np
import rospy
from vslam.msg import Viz
from nav_msgs.msg import OccupancyGrid

from common import send, SERVER
  
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(SERVER)

dataInfo = {
  'id': 'canvas0',
  'streamType': 'customed',
  'dataType': 'json',
  'sequence': 0,
  'timestamp': 1
}

def onGMapData(msg):
  jsonData = {
    'gmap': {
      'info': {
        'resolution': msg.info.resolution,
        'width': msg.info.width,
        'height': msg.info.height,
        'location': {
          'x': msg.info.origin.position.x,
          'y': msg.info.origin.position.y,
          'z': msg.info.origin.position.z
        }
      },
      'data': msg.data
    }
  }
  send(s, dataInfo, json.dumps(jsonData), 'string')

  dataInfo['sequence'] += 1
  dataInfo['timestamp'] += 1

def onVSlamData(msg):
  # get json data
  jsonData = {
    'observers': {},
    'marks': []
  }
  jsonData['observer'] = {
    'id': msg.key_frame.key_frame_id,
    'location': {
      'x': msg.key_frame.camera_pose.position.x, 
      'y': msg.key_frame.camera_pose.position.y, 
      'z': msg.key_frame.camera_pose.position.z
    },
    'orientation': {
      'x': msg.key_frame.camera_pose.orientation.x,
      'y': msg.key_frame.camera_pose.orientation.y,
      'z': msg.key_frame.camera_pose.orientation.z,
      'w': msg.key_frame.camera_pose.orientation.w
    }
  }
  # for data in msg.key_frame:
  #   jsonData['observers'].append({
  #     'location': {
  #       'x': data.camera_pose.position.x, 
  #       'y': data.camera_pose.position.y, 
  #       'z': data.camera_pose.position.z
  #     }, 
  #     'orientation': {
  #       'x': data.camera_pose.orientation.x,
  #       'y': data.camera_pose.orientation.y,
  #       'z': data.camera_pose.orientation.z,
  #       'w': data.camera_pose.orientation.w
  #     }
  #   })
  for data in msg.map_points:
    jsonData['marks'].append({
      'id': data.card_id,
      'code': data.code_id,
      'current' :data.current,
      'location': {
        'x': data.card_pose.position.x,
        'y': data.card_pose.position.y, 
        'z': data.card_pose.position.z
      }
    })
  send(s, dataInfo, json.dumps(jsonData), 'string')

  dataInfo['sequence'] += 1
  dataInfo['timestamp'] += 1

if __name__ == "__main__":

  rospy.init_node('vizMarkSender', anonymous=True)
  rospy.Subscriber('slam_viz', Viz, onVSlamData)
  rospy.Subscriber('map', OccupancyGrid, onGMapData)

  rospy.spin()

  s.close()
