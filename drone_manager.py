#!/usr/bin/env python

from pymavlink import mavutil

class DroneManager():
  def __init__(self, isUDPConnect = False):
    self.protocol = 'udpin'
    self.ipaddr = 'localhost'
    self.port = '14550'
    self.connect_type = 'com9'
    self.isUDPConnect = isUDPConnect
    self.the_connection = None
    self.abs_distance = [0, 0, 0] # x, y, z (m)
    self.abs_attitude = [0, 0, 0] # roll, pitch, yaw (rad)
    self.ground_speed = 5         # 速度 m/s
    self.altitude = 0             # 高度 m
    
  
  def connect(self):
    if self.isUDPConnect:
      # Start a connection listening on a UDP port
      self.the_connection = mavutil.mavlink_connection(self.protocol + ':' + self.ipaddr + ':' + self.port)
      self.the_connection.wait_heartbeat()
    else:
      self.the_connection = mavutil.mavlink_connection(self.connect_type, baud=57600)
      self.the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (self.the_connection.target_system, self.the_connection.target_component))
    return self.the_connection
    
  def arm(self, arm_flag):
    # リモートシステムの起動
    # arm_flag : 1 = アーム、0 = ディスアーム
    self.the_connection.mav.command_long_send(
      self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_flag, 0, 0, 0, 0, 0, 0)

    # コマンドの実行結果や失敗した際の詳細情報を受信する
    msg = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
    print(msg)
  
  def takeoff(self):
    # リモートシステムを離陸させる
    # 離陸高度は10mに設定
    self.the_connection.mav.command_long_send(
      self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, self.altitude)

    msg = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
    print(msg)
  
  def receive_msg(self, msg_type):
    # type
    #  ATTITUDE: 姿勢情報メッセージを受信する
    #  LOCAL_POSITION_NED: 位置情報メッセージを受信する
    #  NAV_CONTROLLER_OUTPUT: 制御情報（ナビゲーション位置とコントローラーの状態）を受信する
    #  COMMAND_ACK: コマンドの実行結果や失敗した際の詳細情報を受信する
    if(msg_type == ''):
      msg_type = None
    msg = self.the_connection.recv_match(type=msg_type, blocking=True)
    return msg
  
  def move(self, diff_pos):
    # 移動
    # diff_pos : 移動量（x, y, z）の
    # type_maskオプション : 位置情報のみの送信（0b110111111000）を指定
    
    # ドローンの現在位置を取得
    current_pos = self.the_connection.location()
    new_pos = current_pos
    new_pos.x += diff_pos[0]
    new_pos.y += diff_pos[1]
    new_pos.z += diff_pos[2]

    command = mavutil.mavlink.set_position_target_local_ned_encode(
      0,                                    # time_boot_ms (not used)
      self.the_connection.target_system,    # target system
      self.the_connection.target_component, # component
      mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
      0b110111111000,                  # type_mask (only positions enabled)
      new_pos.x, new_pos.y, new_pos.z,      # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
      0, 0, 0,                              # x, y, z velocity in m/s  (not used)
      0, 0, 0,                              # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
      0, 0)                                 # yaw, yaw_rate (not used)
    self.send_command(command)
        
  def change_yaw(self, yaw, yaw_rate, clockwise):
    # ヨーの変更
    # yaw : ヨーの変更量（度）
    # yaw_rate : ヨーレート（度/秒）
    # clockwise : 1 = 時計回り、-1 = 反時計回り

    command = self.the_connection.mav.command_long_encode(
      self.the_connection.target_system,      # system id
      self.the_connection.target_component,   # component id
      mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
      0,          # confirmation
      yaw,        # param1, direction in degrees
      yaw_rate,   # param2, speed in degrees/sec
      clockwise,  # param3, direction: 1 = clockwise, -1 = counterclockwise
      1,          # param4, relative offset: 1 = relative, 0 = absolute
      0,          # param5, latitude in degrees
      0,          # param6, longitude in degrees
      0)          # param7, altitude in meters
    self.send_command(command)


  def goto_waypoint(self, waypoint_pos, waypoint_altitude):
    # waypoint(-35.3629849, 149.1649185）の地点へ移動
    command = mavutil.mavlink.set_position_target_global_int(
      0,                          # time_boot_ms (not used)
      self.the_connection.target_system,
      self.the_connection.target_component,
      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
      0b110111111000,             # type_mask (only speeds enabled)
      waypoint_pos.lat * 10 ** 7, # latitude in degrees * 1E7
      waypoint_pos.lon * 10 ** 7, # longitude in degrees * 1E7
      waypoint_altitude,          # altitude in meters
      0, 0, 0,                    # velocity x, y, z
      0, 0, 0,                    # acceleration x, y, z
      0, 0)                       # yaw, yaw_rate
    self.send_command(command)

  def change_speed(self):
    # スピードの設定 
    # GroundSpeed（param1:0）で、目標速度を（param2）に指定
    command = self.the_connection.mav.command_long_encode(
      self.the_connection.target_system,
      self.the_connection.target_component,
      mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
      0,                  # confirmation
      0,                  # param 1 speed type (0,1=Groundspeed, 2=Climb speed, 3=Descent speed)
      self.ground_speed,  # param 2 ground speed in m/s
      0,                  # param 3 throttle value (-1 indicates no change)
      0, 0, 0, 0)         # param 4 - 7 not used
    self.send_command(command)

  def send_command(self, command):
    # コマンドの送信
    # command : 送信するコマンド
    self.the_connection.mav.send(command)