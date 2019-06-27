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

	msg = {
		'observer': [
			{
				'location': { 'x': 1, 'y': 2, 'z': 1}
			}
		],
		'marks': [
			{
				'id': 'mark1',
				'location': {'x': 0, 'y': 2, 'z': 1},
				'code': '0101001'
			}
		]
	}
	send(s, dataInfo, json.dumps(msg), 'string')

	time.sleep(30 / 1000)
	dataInfo['sequence'] += 1
	dataInfo['timestamp'] += 1

	msg = {
		'observer': [{
			'location': { 'x': 2, 'y': 2.5, 'z': 1}
		}],
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
