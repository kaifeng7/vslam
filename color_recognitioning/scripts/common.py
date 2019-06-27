SERVER = ("192.168.3.223", 9000)
TIME_INTERVAL = 30
HEADER_SIZE = 36

def copyByte(src, target, targetOffset, srcStart, srcEnd):
  i = srcStart
  j = targetOffset
  maxLen = min(len(target) - targetOffset, srcEnd - srcStart, len(src) - srcStart)
  while i < srcStart + maxLen:
    target[j] = src[i]
    j += 1
    i += 1

def wrapData(info, data, dataType):
  size = HEADER_SIZE + len(data)
  b = bytearray(size)
  copyByte(bytearray(info['id'], 'utf-8'), b, 0, 0, 8)
  copyByte(bytearray(info['streamType'], 'utf-8'), b, 8, 0, 8)
  copyByte(bytearray(info['dataType'], 'utf-8'), b, 16, 0, 8)
  copyByte(len(data).to_bytes(4, byteorder='big'), b, 24, 0, 8)
  copyByte(info['sequence'].to_bytes(4, byteorder='big'), b, 28, 0, 8)
  copyByte(info['timestamp'].to_bytes(4, byteorder='big'), b, 32, 0, 8)
  if dataType == 'string':
    copyByte(bytearray(data, 'utf-8'), b, 36, 0, len(data))
  else:
    copyByte(bytearray(data), b, 36, 0, len(data))
  return b

def send(s, info, data, dataType):
  s.send(wrapData(info, data, dataType))

