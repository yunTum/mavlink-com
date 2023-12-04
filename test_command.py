from pymavlink import mavutil

class ControlDrone:
  def __init__(self, isUDPConnect = False, com_port='com9', port='14550'):
    self.protocol = 'udpin'
    self.ipaddr = '0.0.0.0'
    self.port = port
    self.com_port = com_port
    self.isUDPConnect = isUDPConnect
    self.the_connection = None
    self.diffrence_foward = 0
    self.hedding = 0
    self.yaw = 0
    self.ground_speed = 5
  
  def connect(self):
    if self.isUDPConnect:
      # Start a connection listening on a UDP port
      self.the_connection = mavutil.mavlink_connection(self.protocol + ':' + self.ipaddr + ':' + self.port)
    else:
      self.the_connection = mavutil.mavlink_connection(self.com_port, baud=57600)
    self.the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (self.the_connection.target_system, self.the_connection.target_component))
    while True:
      msg = self.the_connection.recv_match(type=None, blocking=True)
      # msg = self.the_connection.recv_match(type=None, blocking=True).to_dict()
      # msg = self.the_connection.messages["MAV"].to_dict()
      print(msg)
  
  def arm(self):
    # リモートシステムをアーム（起動）
    # param 1 : 1 = アーム、0 = ディスアーム
    self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    # コマンドの実行結果や失敗した際の詳細情報を受信する
    msg = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
    print(msg)

  def disarm(self):
    # リモートシステムをディスアーム（シャットダウン）
    self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

    msg = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
    print(msg)
  
  def takeoff(self):
    # リモートシステムを離陸させる
    # 離陸高度は10mに設定
    self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)

    msg = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
    print(msg)