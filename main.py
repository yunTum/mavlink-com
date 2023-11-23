from pymavlink import mavutil

class ControlDrone:
  def __init__(self):
    self.protocol = 'udpin'
    self.ipaddr = '0.0.0.0'
    self.port = '14540'
    self.diffrence_foward = 0
    self.hedding = 0
    self.yaw = 0
    self.ground_speed = 5
  
  def connect(self):
    # Start a connection listening on a UDP port
    self.the_connection = mavutil.mavlink_connection(self.protocol + ':' + self.ipaddr + ':' + self.port)
    self.the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (self.the_connection.target_system, self.the_connection.target_component))
    try:
      msg = self.the_connection.recv_match(type='ATTITUDE', blocking=True)
      print(msg)
    except:
      pass
  
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

  def forward(self):
    # （10mの高度まで離陸させた後）高度を10mで保ったまま、10m前進させる
    # type_maskオプション : 位置情報のみの送信（0b110111111000）を指定
    self.diffrence_foward += 10
    self.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_messege(10, self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), self.diffrence_foward, 0, -10, 0, 0, 0, 0, 0, 0, 0, 0))

    # NAV_CONTROLLER_OUTPUT : 制御情報（ナビゲーション位置とコントローラーの状態）を受信
    # 出力される「wp_dist」フィールドの値が waypoint（目標地点）までの距離を表す -> 目標地点に到達したかを確認
    while 1:
      msg = self.the_connection.recv_match(
          type='NAV_CONTROLLER_OUTPUT', blocking=True
      )
      print(msg)
      
  def change_hedding(self):
    # 指定した位置からヘディングを90度（1.57ラジアン）回転させる
    self.hedding += 1.57
    self.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_messege(10, self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b100111111000), self.diffrence_foward, 0, -10, 0, 0, 0, 0, 0, 0, self.hedding, 0))

    while 1:
        msg = self.the_connection.recv_match(
            type='LOCAL_POSITION_NED', blocking=True
        )
        print(msg)
        
  def change_yaw(self):
    # 指定した位置から、さらに毎秒0.5ラジアンでのヨーレートを設定
    self.yaw += 0.5
    self.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_messege(10, self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b010111111000), self.diffrence_foward, 0, -10, 0, 0, 0, 0, 0, 0, self.hedding, self.yaw))

    while 1:
        msg = self.the_connection.recv_match(
            type='LOCAL_POSITION_NED', blocking=True
        )
    print(msg)
  
  def goto_waypoint(self):
    # waypoint(-35.3629849, 149.1649185）の地点へ移動
    self.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_messege(10, self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, int(0b110111111000), int(-35.3629849 * 10 ** 7), int(149.1649185 * 10 ** 7), -10, 0, 0, 0, 0, 0, 0, 0, 0))

    while 1:
        msg = self.the_connection.recv_match(
            type='LOCAL_POSITION_NED', blocking=True
        )
        print(msg)

  def change_speed(self):
    # スピードの設定 
    # GroundSpeed（param1:0）で、目標速度を（param2）に指定
    self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, self.ground_speed, 0, 0, 0, 0, 0)

def main():
  drone = ControlDrone()
  drone.connect()
  drone.arm()
  drone.takeoff()
  drone.disarm()

if __name__ == '__main__':
    main()

# # 5. yawの設定
# # ターゲットヘディングを時計回りに45度回転させる（ヨーの変更速度は25度/秒に設定）
# the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 45, 25, 0, 0, 0, 0, 0)

# # 制御状態をモニタリング
# while 1:
#     msg = the_connection.recv_match(
#         type='NAV_CONTROLLER_OUTPUT', blocking=True
#     )
#     print(msg)


# # 5-2
# # 5-1のヘディングから、さらに時計回りに45度回転
# the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 45, 25, 0, 1, 0, 0, 0)

# while 1:
#     msg = the_connection.recv_match(
#         type='NAV_CONTROLLER_OUTPUT', blocking=True
#     )
#     print(msg)


# # 5-3
# # 5-2のヘディングから、反時計回りに45度回転
# the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 45, 25, -1, 1, 0, 0, 0)

# while 1:
#     msg = the_connection.recv_match(
#         type='NAV_CONTROLLER_OUTPUT', blocking=True
#     )
#     print(msg)
