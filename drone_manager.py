#!/usr/bin/env python

from pymavlink import mavutil
import utilities as util

class DroneManager():
  def __init__(self, isUDPConnect = False, com_port='com9', port='14550'):
    # self.protocol = 'udpin'
    self.protocol = 'tcp'
    self.ipaddr = '127.0.0.1'
    self.port = port
    self.connect_type = com_port
    self.isUDPConnect = isUDPConnect
    self.the_connection = None
    self.state = 'disconnected'
    self.local_pos = [0, 0, 0]    # x, y, z (m)
    self.local_att = [0, 0, 0] # roll, pitch, yaw (rad)
    self.ground_speed = 5         # 速度 m/s
    self.altitude = 0             # 高度 m
    # https://mavlink.io/en/messages/common.html#enums
    self.mav_mode = mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED
    
  def connect(self):
    if self.isUDPConnect:
      # Start a connection listening on a UDP port
      self.the_connection = mavutil.mavlink_connection(self.protocol + ':' + self.ipaddr + ':' + self.port)
      self.the_connection.wait_heartbeat()
      self.state = 'connected'
    else:
      self.the_connection = mavutil.mavlink_connection(self.connect_type, baud=57600)
      self.state = 'connected'
    print("Heartbeat from system (system %u component %u)" % (self.the_connection.target_system, self.the_connection.target_component))
    return self.the_connection

  def disconnect(self):
    if self.the_connection != None:
      self.the_connection.close()
      self.the_connection = None
      print('disconnected', self.the_connection)
      self.state = 'disconnected'
  
  def get_status(self):
    return self.state
    
  def arm(self, arm_flag, force=False):
    # リモートシステムの起動
    # arm_flag : 1 = アーム、0 = ディスアーム
    # force : True = 強制的にアーム
    self.the_connection.mav.command_long_send(
      self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_flag, force, 0, 0, 0, 0, 0)
  
  def takeoff(self, altitude):
    self.altitude = altitude
    # リモートシステムを離陸させる
    # 離陸高度は10mに設定
    self.the_connection.mav.command_long_send(
      self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, self.altitude)

    # msg = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
    # print(msg)
    # return msg
  
  def land(self):
    # リモートシステムを着陸させる
    self.the_connection.mav.command_long_send(
      self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
  
  def receive_msg(self):
    # type
    #  ATTITUDE: 姿勢情報メッセージを受信する
    #  LOCAL_POSITION_NED: 位置情報メッセージを受信する
    #  NAV_CONTROLLER_OUTPUT: 制御情報（ナビゲーション位置とコントローラーの状態）を受信する
    #  COMMAND_ACK: コマンドの実行結果や失敗した際の詳細情報を受信する
    msg = self.the_connection.recv_match(type=None, blocking=True)
    return msg
  
  def wait_heartbeat(self):
    self.the_connection.wait_heartbeat()
  
  def set_local_pos(self, current_pos):
    # 現在位置の設定
    # current_pos : 現在位置（x, y, z）
    self.local_pos = current_pos
    
  def get_local_pos(self):
    # 現在位置の取得
    return self.local_pos
  
  def move(self, diff_pos, isBodyLocal=True):
    # 移動
    # cnt_pos : 現在位置（x, y, z）
    # diff_pos : 移動量（x, y, z）
    # isBodyLocal : True = ボディ座標系、False = ローカル座標系
    new_pos = [0, 0, 0]
    frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED
    if isBodyLocal:
      new_pos[0] = diff_pos[0]
      new_pos[1] = diff_pos[1]
      new_pos[2] = diff_pos[2]
      frame = mavutil.mavlink.MAV_FRAME_BODY_NED
    else:
      new_pos[0] = self.local_pos[0] + diff_pos[0]
      new_pos[1] = self.local_pos[1] + diff_pos[1]
      new_pos[2] = self.local_pos[2] + diff_pos[2]

    command = self.the_connection.mav.set_position_target_local_ned_encode(
      0,                    # time_boot_ms (not used)
      self.the_connection.target_system,    # target system
      self.the_connection.target_component, # component
      frame,                                # frame
      0b110111111000,                  # type_mask (only positions enabled)
      new_pos[0], new_pos[1], new_pos[2],      # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
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


  def set_waypoint(self, waypoint_pos, waypoint_altitude):
    # waypoint(-35.3629849, 149.1649185）の地点へ移動
    latitude = int(float(waypoint_pos[0]) * 1E7)
    longitude = int(float(waypoint_pos[1]) * 1E7)
    command = self.the_connection.mav.set_position_target_global_int_encode(
      0,                          # time_boot_ms (not used)
      self.the_connection.target_system,
      self.the_connection.target_component,
      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
      0b110111111000,             # type_mask (only speeds enabled)
      latitude,                   # latitude in degrees * 1E7
      longitude,                  # longitude in degrees * 1E7
      waypoint_altitude,          # altitude in meters
      0, 0, 0,                    # velocity x, y, z
      0, 0, 0,                    # acceleration x, y, z
      0, 0)                       # yaw, yaw_rate
    self.send_command(command)

  def change_speed(self, speed=5):
    self.ground_speed = speed
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

  def mode_change(self, flag=True, mode_enable=mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, flight_mode=0):
    # モードの変更
    # flag : True = モードを有効化、False = モードを無効化
    # mode_enable : 追加or削除するモードのフラグ
    # https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG
    # flight_mode : フライトモード
    # https://ardupilot.org/copter/docs/parameters.html#fltmode1
    # フライトモード一覧
    # 0 : STABILIZE
    # 1 : ACRO
    # 2 : ALT_HOLD
    # 3 : AUTO
    # 4 : GUIDED
    # ...
    if flag:
      mode_enable = mode_enable | flag
    else:
      mode_enable = mode_enable & ~flag
      
    # # print('mode', mode)
    self.the_connection.mav.command_long_send(
      self.the_connection.target_system, self.the_connection.target_component, 176, 0, mode_enable, flight_mode, 0, 0, 0, 0, 0)
  
  def request_data_stream(self, interval):
    # データストリームのリクエスト(Heartbeatの後に実行する必要あり)
    # interval : データストリームの送信間隔（Hz）
    self.the_connection.mav.request_data_stream_send(
      self.the_connection.target_system, self.the_connection.target_component, 0, interval, 1)

  def send_command(self, command):
    # コマンドの送信
    # command : 送信するコマンド
    self.the_connection.mav.send(command)
  
  def get_glb_location(self):
    # 現在位置の取得
    return self.the_connection.location()