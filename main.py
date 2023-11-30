#!/usr/bin/env python

import sys
import gui
import drone_manager
import test_command

def main():
  args = sys.argv
  if len(args) > 1:
    if args[1] == 'test':
      test_drone = test_command.ControlDrone(isUDPConnect=True)
      test_drone.connect()
  else:
    drone = drone_manager.DroneManager(isUDPConnect=False)
    controller = gui.Controller(drone)
    controller.create_window()

if __name__ == '__main__':
  main()