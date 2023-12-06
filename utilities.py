import numpy as np
import serial.tools.list_ports

def rad2deg(radian):
  return radian * 180 / np.pi

def deg2rad(degree):
  return degree * np.pi / 180

def rad_speed2deg_speed(rad_speed):
  return rad_speed * 180 / np.pi

# COMポートの検索
def search_com():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(port.description, ': ', port.device)
        if port.device == '/dev/ttyACM0':
            return port.device
    return None
  
def mav_result(num):
  if num == 0:
    return 'ACCEPTED'
  elif num == 1:
    return 'TEMPORARILY_REJECTED'
  elif num == 2:
    return 'DENIED'
  elif num == 3:
    return 'UNSUPPORTED'
  elif num == 4:
    return 'FAILED'
  elif num == 5:
    return 'IN_PROGRESS'
  elif num == 6:
    return 'CANCELLED'
  elif num == 7:
    return 'LONG_ONLY'
  elif num == 8:
    return 'INT_ONLY'
  else:
    return 'UNSUPPORTED_MAV_FRAME'

def mav_mode(num):
  if num == 80:
    return 'STABILIZE_DISARMED'
  elif num == 208:
    return 'STABILIZE_ARMED'
  elif num == 64:
    return 'MANUAL_DISARMED'
  elif num == 192:
    return 'MANUAL_ARMED'
  elif num == 88:
    return 'GUIDED_DISARMED'
  elif num == 216:
    return 'GUIDED_ARMED'
  elif num == 92:
    return 'AUTO_DISARMED'
  elif num == 220:
    return 'AUTO_ARMED'
  elif num == 66:
    return 'TEST_DISARMED'
  elif num == 194:
    return 'TEST_ARMED'

def base_mode(mode):
  mask = 0b00000001
  return mode & ~mask

def enable_check(mode):
  mask = 0b0000001
  return mode & mask