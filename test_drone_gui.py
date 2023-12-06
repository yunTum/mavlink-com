#!/usr/bin/env python
import threading
import PySimpleGUI as sg
import utilities as util
# 別ウィンドウでモーターのテストとキャリブレーションを行うGUI

class PreController:
  def __init__(self):
    # self.drone_manager = drone_manager
    self.create_window()

  def create_window(self):
    sg.theme('DarkAmber')
    arm_frame = sg.Frame('',
      [
        [sg.Text('Moter Test')],
        [sg.Button('Moter1', key='-MOTER1-')],
        [sg.Button('Moter2', key='-MOTER2-')],
        [sg.Button('Moter3', key='-MOTER3-')],
        [sg.Button('Moter4', key='-MOTER4-')],
        [sg.Button('All Moter', key='-ALLMOTER-')],
    ])
    accel_frame = sg.Frame('',
      [
        [sg.Text('Accel Calibration')],
        [sg.Button('Average', key='-CALCACCELAVG-'), sg.Text('Average:'), sg.Text('0', key='-GETACCELAVG-')],
        [sg.InputText(key='-ACCELAVG-', size=(10, 5)), sg.Button('SET', key='-SETACCELAVG-')]
    ])
    layout = [
      [sg.Text('Pre Controller')],
      [arm_frame, accel_frame],
    ]
    self.window = sg.Window('Pre Controller', layout)
    
  def run(self):
    
    while True:
      event, values = self.window.read(timeout=100)
      if event == sg.WIN_CLOSED or event == '-EXIT-':
        break
      elif event == '-ARM-':
        self.drone_manager.arm(1)
      elif event == '-DISARM-':
        self.drone_manager.arm(0)
      elif event == '-TAKEOFF-':
        self.drone_manager.takeoff(10)
      elif event == '-LAND-':
        self.drone_manager.land()
    self.window.close()
    
  
def main():
  pre_controller = PreController()
  pre_controller.run()

if __name__ == '__main__':
  main()