#!/usr/bin/env python
import threading
import PySimpleGUI as sg
import utilities as util

class Controller:
  def __init__(self, drone_manager):
    self.drone_manager = drone_manager
    self.state = 'init'
    self.arm_status = 'disarmed'
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
      ], size=(200,350)
    )
    udp_frame = sg.Frame('',
      [
        [sg.Text('Socket State')],
        [sg.Text('Socket:'), sg.Text('disconneted', key='-SOCKET-', size=(10,1))],
      ], size=(200,150)
    )
    connect_frame = sg.Frame('',
      [
        [sg.Text('Connect')],
        [sg.Button('Connect', key='-CONNECT-'), sg.Button('Disconnect', key='-DISCONNECT-', disabled=True)],
      ], pad=((20, 0), ( 0, 10))
    )
    logging_frame = sg.Frame('',
      [
        [sg.Multiline(size=(150,10), key='-LOGGING-', disabled=True, autoscroll=True)],
      ], pad=((20, 0), ( 0, 10))
    )
    arm_frame = sg.Frame('',
      [
        [sg.Text('Armming')],
        [sg.Text('Arm:'), sg.Text('disarmed', key='-ARMSTATUS-', size=(10,1))],
        [sg.Button('ARM', key='-ARM-'), sg.Button('DISARM', key='-DISARM-', disabled=True)],
      ], size=(200,100), pad=((20, 0), ( 0, 10))
    )                                  
    flight_flame = sg.Frame('',
      [
        [sg.Text('Flight')],
        [sg.InputText('0', size=(3,1), key='-SETALTITUDE-'), sg.Text('m', size=(1,1))],
        [sg.Button('TAKEOFF', key='-TAKEOFF-'), sg.Button('LAND', key='-LAND-')],
      ], size=(200,100), pad=((20, 0), ( 0, 10))
    )
    
    move_frame = sg.Frame('',
      [
        [sg.Button('L', size=(5,2), key='-LEFTROLL-'), sg.Button('↑', size=(5,2), key='-FRONT-'), sg.Button('R', size=(5,2), key='-RIGHTROLL-')],
        [sg.Button('←', size=(5,2), key='-LEFT-'), sg.Button('↓', size=(5,2), key='-BACK-'), sg.Button('→', size=(5,2), key='-RIGHT-')],
      ], size=(200,200), pad=((20, 0), ( 0, 10))
    )
    
    state_frame = sg.Frame('',
      [
        [flight_state_frame],
        [udp_frame],
      ], relief=sg.RELIEF_FLAT
    )
    
    control_frame = sg.Frame('',
      [
        [connect_frame],
        [arm_frame],
        [flight_flame],
        [move_frame],
      ], relief=sg.RELIEF_FLAT
    )
    
    layout =  [ 
                [state_frame, control_frame],
                [logging_frame]
              ]

    self.window = sg.Window('GuiController', layout, resizable=True, finalize=True, size=(800, 600))
  
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
      # elif event == '-LEFT-':
      #   self.move_left()
      # elif event == '-RIGHT-':
      #   self.move_right()
      # elif event == '-FRONT-':
      #   self.move_front()
      # elif event == '-BACK-':
      #   self.move_back()
      # elif event == '-LEFTROLL-':
      #   self.move_leftroll()
      # elif event == '-RIGHTROLL-':
      #   self.move_rightroll()
      elif event == '-ARM-':
        self.arm()
      elif event == '-DISARM-':
        self.disarm()
      else:
        pass
    self.window.close()
    
  def connect(self):
    try:
      self.drone_manager.connect()
      self.window['-CONNECT-'].update(disabled=True)
      self.window['-DISCONNECT-'].update(disabled=False)
      if (self.drone_manager.get_status() == 'connected'):
        print('connected')
        self.window['-SOCKET-'].update('connected')
        self.state = 'connected'
        self.window['-LOGGING-'].print('connected')
        recv_msg = self.drone_manager.mode_change()
        self.log_text += str(recv_msg) + '\n'
        self.window['-LOGGING-'].print(self.log_text)
        self.drone_manager.wait_heartbeat()
        self.recv_msg_thread.start()
      else:
        self.window['-SOCKET-'].update('disconnected')
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
      self.window['-SOCKET-'].update('disconnected')
      self.window['-LOGGING-'].print('disconnected')
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
      elif msg_dict['mavpackettype'] == 'AHRS2':
        self.window['-ALTITUDE-'].update(str(msg_dict['altitude']))
      elif msg_dict['mavpackettype'] == 'BATTERY_STATUS':
        self.window['-BATTERY-'].update(str(msg_dict['battery_remaining']))
      elif msg_dict['mavpackettype'] == 'VFR_HUD':
        self.window['-GROUNDSPEED-'].update(str(msg_dict['groundspeed']))
      elif msg_dict['mavpackettype'] == 'GLOBAL_POSITION_INT':
        self.window['-LATITUDE-'].update(str(msg_dict['lat']))
        self.window['-LONGITUDE-'].update(str(msg_dict['lon']))
      elif msg_dict['mavpackettype'] == 'COMMAND_ACK':
        command_id = str(msg_dict['command'])
        result = str(msg_dict['result'])
        update_text = 'CommandID: ' + command_id + ', ' + ' Result: ' + result
        self.log_text += update_text + '\n'
        self.window['-LOGGING-'].update(self.log_text)
        if (command_id == '400' and result == '0' and self.arm_flag == False):
          self.window['-ARMSTATUS-'].update('armed')
          self.window['-ARM-'].update(disabled=True)
          self.window['-DISARM-'].update(disabled=False)
          self.arm_flag = True
        elif (command_id == '400' and result == '0' and self.arm_flag == True):
          self.window['-ARMSTATUS-'].update('disarmed')
          self.window['-ARM-'].update(disabled=False)
          self.window['-DISARM-'].update(disabled=True)
          self.arm_flag = False
      
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
        self.drone_manager.arm(arm_flag=1)
        self.log_text += 'arm\n'
        self.window['-LOGGING-'].print(self.log_text)
      except Exception as e:
        print(e)
  
  def disarm(self):
    if self.state == 'connected':
      try:
        self.drone_manager.arm(arm_flag=0)
        self.log_text += 'disarm\n'
        self.window['-LOGGING-'].print(self.log_text)
      except Exception as e:
        print(e)