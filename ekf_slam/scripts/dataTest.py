import socket
import json
import time

from common import send, SERVER

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(SERVER)

dataInfo = {
  'id': 'canvas0',
  'streamType': 'customed',
  'dataType': 'json',
  'sequence': 1,
  'timestamp': 1
}

if __name__ == "__main__":
  
	# msg = {
	# 	'gmap': {
	# 		'info': {
	# 			'resolution': 1,
	# 			'width': 4,
	# 			'height': 4,
	# 			'location': { 'x': 0, 'y': 0, 'z': 0 }
	# 		},
	# 		'data': [0, 10, 100, 5, 0, 10, 100, 5, 0, 10, 100, 5, 0, 10, 100, 5]
	# 	}
	# }
	# send(s, dataInfo, json.dumps(msg), 'string')

	msg = {
		'observer': 
		{
			'id': 0,
			'location': { 'x': 1, 'y': 2, 'z': 1},
			'orientation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 }
		},
		'marks': [
			{
				'id': 'mark1',
				'location': {'x': 0, 'y': 2, 'z': 1},
				'code': '0101001'
			}, 
			{
				'id': 'mark2',
				'location': {'x': 1, 'y': 4, 'z': 1},
				'code': '0101001'
			}
		]
	}
	send(s, dataInfo, json.dumps(msg), 'string')

	time.sleep(30 / 1000)
	dataInfo['sequence'] += 1
	dataInfo['timestamp'] += 1

	msg = {
		'observer': 
		{
			'id': 1,
			'location': { 'x': 2, 'y': 2.5, 'z': 1},
			'orientation': { 'x': 0.6, 'y': 0, 'z': 0, 'w': 0.8 }
		},
		'marks': [
			{
				'id': 'mark1',
				'location': {'x': 0, 'y': 2, 'z': 1},
				'code': '0101001'
			}
		]
	}
	send(s, dataInfo, json.dumps(msg), 'string')

	s.close()
