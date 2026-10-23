#!/usr/bin/python3

import sys
import signal
from PyQt5 import QtWidgets, QtCore
from robot_starter_interface import RobotInterface

"""
 @author: Mehmet Kahraman
 @date: 18.03.2024
 @about: Robot Starter gui interface for orion 5 arm
"""
        
if __name__ == "__main__":
    
    def sigint_handler(*args):
        print("\n Interface Closed with Interrupt. \n")
        QtWidgets.QApplication.quit()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, sigint_handler)

    try:
        app = QtWidgets.QApplication([])
        
        timer = QtCore.QTimer()
        timer.start(400)
        timer.timeout.connect(lambda: None)
        
        window = RobotInterface("orion5_interface")
        app.exec()
        
    except:
        print("\n ROS Node stopped. \n")