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