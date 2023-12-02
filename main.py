#!/usr/bin/env python

import argparse
import gui
import drone_manager
import test_command
import search_com

def main():
  parser = argparse.ArgumentParser(description='Drone Controller')
  parser.add_argument('positional', help='positional argument: com or test')
  parser.add_argument('--comport', help='com port')
  parser.add_argument('--udp', help='udp port')
  args = parser.parse_args()
  if args.positional == 'com':
    # search comport
    print('search comport')
    com = search_com.search_com()
    print(com)
  # TODO: 処理フローの整理
  # テストコマンドの実行
  elif args.positional == 'test':
    if args.udp != None:
      udp_port = args.udp
      test_drone = test_command.ControlDrone(isUDPConnect=True, udp_port=udp_port)
      test_drone.connect()
    elif args.comport != None:
      com_port = args.comport
      test_drone = test_command.ControlDrone(isUDPConnect=False, com_port=com_port)
      test_drone.connect()
    else:
      print('the following arguments are required: --comport or --udp')
  # GUIの実行
  else:
    if args.udp != None:
      udp_port = args.udp
      drone = drone_manager.DroneManager(isUDPConnect=True, udp_port=udp_port)
    elif args.comport != None:
      com_port = args.comport
      drone = drone_manager.DroneManager(isUDPConnect=False, com_port=com_port)
    if drone == None:
      return
    controller = gui.Controller(drone)
    controller.create_window()

if __name__ == '__main__':
  main()