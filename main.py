#!/usr/bin/env python

import argparse
import gui
import drone_manager
import test_command
import utilities as util

def main():
  parser = argparse.ArgumentParser(description='Drone Controller')
  parser.add_argument('positional', help='positional argument: com or test')
  parser.add_argument('--comport', help='com port')
  parser.add_argument('--port', help='port')
  args = parser.parse_args()
  if args.positional == 'com':
    # search comport
    print('search comport')
    com = util.search_com.search_com()
    print(com)
  # TODO: 処理フローの整理
  # テストコマンドの実行
  elif args.positional == 'test':
    if args.port != None:
      port = args.port
      test_drone = test_command.ControlDrone(isUDPConnect=True, port=port)
      test_drone.connect()
    elif args.comport != None:
      com_port = args.comport
      test_drone = test_command.ControlDrone(isUDPConnect=False, com_port=com_port)
      test_drone.connect()
    else:
      print('the following arguments are required: --comport or --port')
  # GUIの実行
  else:
    if args.port != None:
      port = args.port
      drone = drone_manager.DroneManager(isUDPConnect=True, port=port)
    elif args.comport != None:
      com_port = args.comport
      drone = drone_manager.DroneManager(isUDPConnect=False, com_port=com_port)
    if drone == None:
      return
    controller = gui.Controller(drone)
    controller.create_window()

if __name__ == '__main__':
  main()