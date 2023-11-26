#!/usr/bin/env python
import time
import threading
import PySimpleGUI as sg

class Controller:
  def __init__(self, drone_manager):
    self.drone_manager = drone_manager
    self.state = 'init'
    self.window = None
    self.recv_msg_thread = threading.Thread(target=self.recv_msg)
    self.create_window()
    self.run()

  def create_window(self):
    sg.theme('DarkGrey5')
    flight_state_frame = sg.Frame('',
      [
        [sg.Text('Fight State')],
        [sg.Text('Battery:', size=(11, 1)), sg.Text('0', key='-BATTERY-', size=(3, 1)), sg.Text('%')],
        [sg.Text('X-Acc:', size=(11, 1)), sg.Text('0', key='-XACC-', size=(3, 1)), sg.Text('m/s^2')],
        [sg.Text('Y-Acc:', size=(11, 1)), sg.Text('0', key='-YACC-', size=(3, 1)), sg.Text('m/s^2')],
        [sg.Text('Z-Acc:', size=(11, 1)), sg.Text('0', key='-ZACC-', size=(3, 1)), sg.Text('m/s^2')],
        [sg.Text('X-Speed:', size=(11, 1)), sg.Text('0', key='-XSPEED-', size=(3, 1)), sg.Text('m/s')],
        [sg.Text('Y-Speed:', size=(11, 1)), sg.Text('0', key='-YSPEED-', size=(3, 1)), sg.Text('m/s')],
        [sg.Text('Z-Speed:', size=(11, 1)), sg.Text('0', key='-ZSPEED-', size=(3, 1)), sg.Text('m/s')],
        [sg.Text('Roll:', size=(11, 1)), sg.Text('0', key='-ROLL-', size=(3, 1)), sg.Text('°')],
        [sg.Text('Pitch:', size=(11, 1)), sg.Text('0', key='-PITCH-', size=(3, 1)), sg.Text('°')],
        [sg.Text('Yaw:', size=(11, 1)), sg.Text('0', key='-YAW-', size=(3, 1)), sg.Text('°')],
        [sg.Text('Barometer:', size=(11, 1)), sg.Text('0', key='-BAROMETER-', size=(3, 1)), sg.Text('m')],
        [sg.Text('Altitude:', size=(11, 1)), sg.Text('0', key='-ALTITUDE-', size=(3, 1)), sg.Text('m')],
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
        [sg.Multiline(size=(40,10), key='-LOGGING-', disabled=True, autoscroll=True)],
      ], pad=((20, 0), ( 0, 10))
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
        [flight_flame],
        [move_frame],
        [logging_frame],
      ], relief=sg.RELIEF_FLAT
    )
    
    layout =  [ 
                [state_frame, control_frame],
              ]

    self.window = sg.Window('Tello Controller', layout, resizable=True, finalize=True, size=(600, 500))
  
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
      else:
        pass
    self.window.close()
    
  def connect(self):
    try:
      connect_obj = self.drone_manager.connect()
      self.window['-CONNECT-'].update(disabled=True)
      self.window['-DISCONNECT-'].update(disabled=False)
      print(connect_obj)
      if (connect_obj is not None):
        print('connected')
        self.window['-SOCKET-'].update('connected')
        self.state = 'connected'
        self.recv_msg_thread.start()
      else:
        self.window['-SOCKET-'].update('disconnected')
        self.state = 'init'
    except Exception as e:
      print(e)
  
  def disconnect(self):
    try:
      self.recv_msg_thread.join()
      self.window['-CONNECT-'].update(disabled=False)
      self.window['-DISCONNECT-'].update(disabled=True)
      self.window['-SOCKET-'].update('disconnected')
      self.state = 'init'
    except Exception as e:
      print(e)
    
  def recv_msg(self):
    while self.state == 'connected':
      msg = self.drone_manager.receive_msg('')
      print(msg)
      