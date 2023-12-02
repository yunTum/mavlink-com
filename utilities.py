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