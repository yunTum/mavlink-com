#!/usr/bin/env python
import threading
import PySimpleGUI as sg
import utilities as util

class Controller:
  def __init__(self, drone_manager):
    self.drone_manager = drone_manager
    self.state = 'init'
    self.window = None
    self.recv_msg_thread = threading.Thread(target=self.recv_msg)
    self.log_text = ''
    self.arm_flag = False
    self.create_window()
    self.run()

  # TODO: ウィジェットの外部化
  def create_window(self):
    sg.theme('DarkGrey5')
    flight_state_frame = sg.Frame('',
      [
        [sg.Text('Fight State')],
        [sg.Text('Battery:', size=(11, 1)), sg.Text('0', key='-BATTERY-', size=(8, 1)), sg.Text('%')],
        [sg.Text('Local-X:', size=(11, 1)), sg.Text('0', key='-LOCALX-', size=(8, 1)), sg.Text('m')],
        [sg.Text('Local-Y:', size=(11, 1)), sg.Text('0', key='-LOCALY-', size=(8, 1)), sg.Text('m')],
        [sg.Text('Local-Z:', size=(11, 1)), sg.Text('0', key='-LOCALZ-', size=(8, 1)), sg.Text('m')],
        [sg.Text('GroundSpeed:', size=(11, 1)), sg.Text('0', key='-GROUNDSPEED-', size=(8, 1)), sg.Text('m/s')],
        [sg.Text('Roll:', size=(11, 1)), sg.Text('0', key='-ROLL-', size=(8, 1)), sg.Text('°')],
        [sg.Text('RollSpeed:', size=(11, 1)), sg.Text('0', key='-ROLLSPEED-', size=(8, 1)), sg.Text('°/s')],
        [sg.Text('Pitch:', size=(11, 1)), sg.Text('0', key='-PITCH-', size=(8, 1)), sg.Text('°')],
        [sg.Text('PitchSpeed:', size=(11, 1)), sg.Text('0', key='-PITCHSPEED-', size=(8, 1)), sg.Text('°/s')],
        [sg.Text('Yaw:', size=(11, 1)), sg.Text('0', key='-YAW-', size=(8, 1)), sg.Text('°')],
        [sg.Text('YawSpeed:', size=(11, 1)), sg.Text('0', key='-YAWSPEED-', size=(8, 1)), sg.Text('°/s')],
        [sg.Text('Barometer:', size=(11, 1)), sg.Text('0', key='-BAROMETER-', size=(8, 1)), sg.Text('m')],
        [sg.Text('Altitude:', size=(11, 1)), sg.Text('0', key='-ALTITUDE-', size=(8, 1)), sg.Text('m')],
        [sg.Text('Latitude:', size=(11, 1)), sg.Text('0', key='-LATITUDE-', size=(8, 1)), sg.Text('°')],
        [sg.Text('Longitude:', size=(11, 1)), sg.Text('0', key='-LONGITUDE-', size=(8, 1)), sg.Text('°')],
      ], size=(250,450)
    )
    drone_state_frame = sg.Frame('',
      [
        [sg.Text('Drone State')],
        [sg.Text('Socket:'), sg.Text('DICONNECTED', key='-SOCKET-', size=(18,1))],
        [sg.Text('Mode:'), sg.Text('STABLE', key='-MODE-', size=(18,1))],
        [sg.Text('Arm:'), sg.Text('DISARMED', key='-ARMSTATUS-', size=(18,1))]
      ], size=(200,150)
    )
    connect_frame = sg.Frame('',
      [
        [sg.Text('Connect')],
        [sg.Button('CONNECT', key='-CONNECT-'), sg.Button('DICONNECTED', key='-DISCONNECT-', disabled=True)],
      ]
    )
    mode_change_frame = sg.Frame('',
      [
        [sg.Text('Mode Change')],
        [sg.Button('STABLE', key='-STABLE-'), sg.Button('GUIDED', key='-GUIDED-'), sg.Button('AUTO', key='-AUTO-')],
      ]
    )
    logging_frame = sg.Frame('',
      [
        [sg.Multiline(size=(150,10), key='-LOGGING-', disabled=True, autoscroll=True)],
      ], pad=((20, 0), ( 0, 10))
    )
    arm_frame = sg.Frame('',
      [
        [sg.Text('Armming')],
        [sg.Button('ARM', key='-ARM-', disabled=True), sg.Button('DISARM', key='-DISARM-', disabled=True), sg.Checkbox('Force', key='-FORCE-')],
      ], size=(200,100)
    )                       
    flight_flame = sg.Frame('',
      [
        [sg.Text('Flight')],
        [sg.InputText('0', size=(3,1), key='-SETALTITUDE-'), sg.Text('m', size=(1,1))],
        [sg.Button('TAKEOFF', key='-TAKEOFF-'), sg.Button('LAND', key='-LAND-')],
      ], size=(200,100)
    )
    speed_frame = sg.Frame('',
      [
        [sg.Text('Change Speed')],
        [sg.InputText('5', size=(4,1), key='-CHANGESPEED-'), sg.Text('m/s'),sg.Button('SET', key='-SETSPEED-')],
      ], size=(200,100)
    )
    waypoint_frame = sg.Frame('',
      [
        [sg.Text('Waypoint')],
        [sg.Text('Latitude:'), sg.InputText('0', size=(18,1), key='-WAYPOINTLATITUDE-', pad=((16,0), (0,0))), sg.Text('°')], 
        [sg.Text('Longitude:'), sg.InputText('0', size=(18,1), key='-WAYPOINTLONGITUDE-', pad=((6,0), (0,0))), sg.Text('°')],
        [sg.Button('SET', key='-WAYPOINT-')],
      ], size=(200,100)
    )
    move_diff_frame = sg.Frame('',
      [
        [sg.Text('Move Diff')],
        [sg.Text('X:'), sg.InputText('5', size=(4,1), key='-MOVEDIFFX-'), sg.Text('m')],
        [sg.Text('Y:'),sg.InputText('5', size=(4,1), key='-MOVEDIFFY-'), sg.Text('m')],
        [sg.Text('YAW:'),sg.InputText('20', size=(4,1), key='-MOVEYAW-'), sg.Text('°')],
      ], size=(200,100)
    )
    move_frame = sg.Frame('',
      [
        [sg.Button('L', size=(5,2), key='-LEFTROLL-'), sg.Button('↑', size=(5,2), key='-FRONT-'), sg.Button('R', size=(5,2), key='-RIGHTROLL-')],
        [sg.Button('←', size=(5,2), key='-LEFT-'), sg.Button('↓', size=(5,2), key='-BACK-'), sg.Button('→', size=(5,2), key='-RIGHT-')],
      ], size=(200,200)
    )
    
    state_frame = sg.Frame('',
      [
        [flight_state_frame],
      ], relief=sg.RELIEF_FLAT
    )
    first_control_frame = sg.Frame('',
      [
        [drone_state_frame],
        [connect_frame],
        [mode_change_frame],
        [arm_frame],
        [flight_flame],
      ], relief=sg.RELIEF_FLAT
    )
    control_frame = sg.Frame('',
      [
        [speed_frame],
        [waypoint_frame],
        [move_diff_frame],
        [move_frame],
      ], relief=sg.RELIEF_FLAT
    )
    
    layout =  [ 
                [state_frame, first_control_frame, control_frame],
                [logging_frame]
              ]

    self.window = sg.Window('GuiController', layout, resizable=True, finalize=True, size=(800, 700))

  def run(self):
    while True:
      event, values = self.window.read(timeout=100)
      if event == sg.WIN_CLOSED:
        break
      elif event == '-CONNECT-':
        self.connect()
      elif event == '-DISCONNECT-':
        self.disconnect()
      elif event == '-TAKEOFF-':
        self.takeoff()
      elif event == '-LAND-':
        self.land()
      elif event == '-LEFT-':
        self.move_left()
      elif event == '-RIGHT-':
        self.move_right()
      elif event == '-FRONT-':
        self.move_front()
      elif event == '-BACK-':
        self.move_back()
      elif event == '-LEFTROLL-':
        self.move_leftroll()
      elif event == '-RIGHTROLL-':
        self.move_rightroll()
      elif event == '-ARM-':
        self.arm()
      elif event == '-DISARM-':
        self.disarm()
      elif event == '-WAYPOINT-':
        self.set_waypoint()
      elif event == '-STABLE-':
        self.change_mode(mode_enable=16, flight_mode=0)
      elif event == '-GUIDED-':
        self.change_mode(mode_enable=8, flight_mode=4)
      elif event == '-AUTO-':
        self.change_mode(mode_enable=4, flight_mode=3)
      elif event == '-SETSPEED-':
        self.set_speed()
      else:
        pass
    self.window.close()
    
  def connect(self):
    try:
      self.drone_manager.connect()
      self.window['-CONNECT-'].update(disabled=True)
      self.window['-DISCONNECT-'].update(disabled=False)
      if (self.drone_manager.get_status() == 'connected'):
        self.window['-SOCKET-'].update('CONNECTED')
        self.state = 'connected'
        self.window['-LOGGING-'].print('CONNECTED')
        self.drone_manager.wait_heartbeat()
        self.drone_manager.request_data_stream(5)
        self.recv_msg_thread.start()
      else:
        self.window['-SOCKET-'].update('DISCONNECTED')
        self.state = 'init'
    except Exception as e:
      print(e)
  
  def disconnect(self):
    try:
      self.state = 'disconnected'
      if (self.recv_msg_thread.is_alive()):
        self.recv_msg_thread.join()
      self.drone_manager.disconnect()
      self.window['-CONNECT-'].update(disabled=False)
      self.window['-DISCONNECT-'].update(disabled=True)
      self.window['-SOCKET-'].update('DISCONNECTED')
      self.window['-LOGGING-'].print('DISCONNECTED')
    except Exception as e:
      print(e)

  # TODO : 受信処理の整理
  def recv_msg(self):
    while self.state == 'connected':
      msg = self.drone_manager.receive_msg()
      msg_dict = msg.to_dict()
      if msg_dict['mavpackettype'] == 'ATTITUDE':
        roll = util.rad2deg(msg_dict['roll'])
        pitch = util.rad2deg(msg_dict['pitch'])
        yaw = util.rad2deg(msg_dict['yaw'])
        self.window['-ROLL-'].update(str(roll))
        self.window['-PITCH-'].update(str(pitch))
        self.window['-YAW-'].update(str(yaw))
        rollspeed = util.rad_speed2deg_speed(msg_dict['rollspeed'])
        pitchspeed = util.rad_speed2deg_speed(msg_dict['pitchspeed'])
        yawspeed = util.rad_speed2deg_speed(msg_dict['yawspeed'])
        self.window['-ROLLSPEED-'].update(str(rollspeed))
        self.window['-PITCHSPEED-'].update(str(pitchspeed))
        self.window['-YAWSPEED-'].update(str(yawspeed))
      elif msg_dict['mavpackettype'] == 'LOCAL_POSITION_NED':
        self.window['-LOCALX-'].update(str(msg_dict['x']))
        self.window['-LOCALY-'].update(str(msg_dict['y']))
        self.window['-LOCALZ-'].update(str(msg_dict['z']))
        self.drone_manager.local_pos = [msg_dict['x'], msg_dict['y'], msg_dict['z']]
      elif msg_dict['mavpackettype'] == 'AHRS2':
        self.window['-ALTITUDE-'].update(str(msg_dict['altitude']))
      elif msg_dict['mavpackettype'] == 'BATTERY_STATUS':
        self.window['-BATTERY-'].update(str(msg_dict['battery_remaining']))
      elif msg_dict['mavpackettype'] == 'VFR_HUD':
        self.window['-GROUNDSPEED-'].update(str(msg_dict['groundspeed']))
      elif msg_dict['mavpackettype'] == 'GLOBAL_POSITION_INT':
        self.window['-LATITUDE-'].update(str(msg_dict['lat'] / 1e7))
        self.window['-LONGITUDE-'].update(str(msg_dict['lon'] / 1e7))
      elif msg_dict['mavpackettype'] == 'COMMAND_ACK':
        command_id = str(msg_dict['command'])
        result_id = msg_dict['result']
        result = util.mav_result(result_id)
        update_text = 'CommandID: ' + command_id + ', ' + ' Result: ' + result
        self.log_text += update_text + '\n'
        self.window['-LOGGING-'].update(self.log_text)
      elif msg_dict['mavpackettype'] == 'HEARTBEAT' and msg_dict['type'] == 2:
        mode = util.mav_mode(msg_dict['custom_mode'])
        self.window['-MODE-'].update(mode)
        raw_mode = util.base_mode(int(msg_dict['base_mode']))
        mode_arm_str, self.arm_flag = util.mav_mode_arm(raw_mode)
        print(msg_dict)
        self.window['-ARMSTATUS-'].update(mode_arm_str)
        self.window['-ARM-'].update(disabled=self.arm_flag)
        self.window['-DISARM-'].update(disabled=not self.arm_flag)
      
  def takeoff(self):
    if self.state == 'connected':
      if self.window['-SETALTITUDE-'].get() != '':
        altitude = int(self.window['-SETALTITUDE-'].get())
      try:
        self.drone_manager.takeoff(altitude=altitude)
        self.log_text += 'takeoff\n'
        self.window['-LOGGING-'].print(self.log_text)
      except Exception as e:
        print(e)
  
  def land(self):
    if self.state == 'connected':
      try:
        self.drone_manager.land()
        self.log_text += 'land\n'
        self.window['-LOGGING-'].print(self.log_text)
      except Exception as e:
        print(e)
  
  def arm(self):
    if self.state == 'connected':
      try:
        if self.window['-FORCE-'].get():
          self.drone_manager.arm(arm_flag=1, force=True)
        self.drone_manager.arm(arm_flag=1)
        self.log_text += 'arm\n'
        self.window['-LOGGING-'].print(self.log_text)
        self.arm_flag = True
        self.window['-ARMSTATUS-'].update('ARMED')
      except Exception as e:
        print(e)
  
  def disarm(self):
    if self.state == 'connected':
      try:
        if self.window['-FORCE-'].get():
          self.drone_manager.arm(arm_flag=0, force=True)
        self.drone_manager.arm(arm_flag=0)
        self.log_text += 'disarm\n'
        self.window['-LOGGING-'].print(self.log_text)
        self.arm_flag = False
        self.window['-ARMSTATUS-'].update('DISARMED')
      except Exception as e:
        print(e)

        
  def move_front(self):
    if self.state == 'connected':
      try:
        diff = int(self.window['-MOVEDIFFX-'].get())
        self.drone_manager.move(diff_pos=[diff, 0, 0])
        pos = self.drone_manager.get_local_pos()
        print(pos)
        self.log_text += 'move_front\n'
        self.window['-LOGGING-'].print(self.log_text)
        newpos = self.drone_manager.get_local_pos()
        self.log_text += str(newpos) + '\n'
        self.window['-LOGGING-'].print(self.log_text)
      except Exception as e:
        print(e)
  
  def move_back(self):
    if self.state == 'connected':
      try:
        diff = int(self.window['-MOVEDIFFX-'].get())
        self.drone_manager.move(diff_pos=[-diff, 0, 0])
        self.log_text += 'move_back\n'
        self.window['-LOGGING-'].print(self.log_text)
      except Exception as e:
        print(e)
  
  def move_left(self):
    if self.state == 'connected':
      try:
        diff = int(self.window['-MOVEDIFFY-'].get())
        self.drone_manager.move(diff_pos=[0, -diff, 0])
        self.log_text += 'move_left\n'
        self.window['-LOGGING-'].print(self.log_text)
      except Exception as e:
        print(e)
  
  def move_right(self):
    if self.state == 'connected':
      try:
        diff = int(self.window['-MOVEDIFFY-'].get())
        self.drone_manager.move(diff_pos=[0, diff, 0])
        self.log_text += 'move_right\n'
        self.window['-LOGGING-'].print(self.log_text)
      except Exception as e:
        print(e)
        
  def move_leftroll(self):
    if self.state == 'connected':
      try:
        diff = int(self.window['-MOVEYAW-'].get())
        self.drone_manager.change_yaw(diff, 1, -1)
        self.log_text += 'move_leftroll\n'
        self.window['-LOGGING-'].print(self.log_text)
      except Exception as e:
        print(e)
        
  def move_rightroll(self):
    if self.state == 'connected':
      try:
        diff = int(self.window['-MOVEYAW-'].get())
        self.drone_manager.change_yaw(diff, 1, 0)
        self.log_text += 'move_rightroll\n'
        self.window['-LOGGING-'].print(self.log_text)
      except Exception as e:
        print(e)
  
  def set_waypoint(self):
    if self.state == 'connected':
      try:
        latitude = self.window['-WAYPOINTLATITUDE-'].get()
        longitude = self.window['-WAYPOINTLONGITUDE-'].get()
        self.drone_manager.set_waypoint([latitude, longitude], 10)
        self.log_text += 'set_waypoint\n'
        self.window['-LOGGING-'].print(self.log_text)
      except Exception as e:
        print(e)
  
  def change_mode(self, mode_enable, flight_mode):
    if self.state == 'connected':
      try:
        self.drone_manager.mode_change(flag=True, mode_enable=mode_enable, flight_mode=flight_mode)
        self.log_text += 'change_mode\n'
        self.window['-LOGGING-'].print(self.log_text)
      except Exception as e:
        print(e)
        
  def set_speed(self):
    if self.state == 'connected':
      try:
        speed = self.window['-CHANGESPEED-'].get()
        self.drone_manager.change_speed(int(speed))
        self.log_text += 'set_speed:' + speed + 'm/s\n'
        self.window['-LOGGING-'].print(self.log_text)
        if (int(speed) > 10):
          self.log_text += 'WARNING:' + speed +'m/s is too fast limit 10m/s\n'
          self.window['-LOGGING-'].print(self.log_text)
      except Exception as e:
        print(e)