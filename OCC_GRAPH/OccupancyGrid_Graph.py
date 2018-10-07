# -*- coding: utf-8 -*-
"""
This example demonstrates many of the 2D plotting capabilities
in pyqtgraph. All of the plots may be panned/scaled by dragging with
the left/right mouse buttons. Right click on any plot to show a context menu.
"""

HOST = '10.2.124.96'
UDP_PORT = 12346  # The same port as used by the server

# import initExample ## Add path to library (just for examples; you do not need this)

import math
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import socket
import struct


class OccGridPacketReader:
    # def __init__(self):
    def __init__(self):
        self.rows = 0
        self.cols = 0
        self.car_pos_x = 0
        self.car_pos_y = 0
        self.angle_decision = 0


    def read_test(self):
        try:

            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind((HOST, UDP_PORT))

            data, addr = s.recvfrom(30000)
            unpacked_data = struct.unpack(">i", data)
            angel = unpacked_data[0]
            test = int.from_bytes(data[:4], byteorder='big')
            print(test)

        except Exception as e:
            print("EXEC_GUICMD: Parse Error - unable to parse command" + str(e))
            return
    def read_grid(self):
        try:

            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind((HOST, UDP_PORT))
            print('connected %s', HOST)

            ################################################################
            # OCCUPANCY GRID UDP RECEIVE
            ################################################################
            # header = 0x55555555
            # time = datetime.datetime.utcnow()
            # gridRows = 300
            # gridColumns = 50
            # carPositionX = 25
            # carPositionY = 0
            # angleOfDecision = +- 45
            # checkSum = 999
            # gridArray = *

            data, addr = s.recvfrom(30000)
            packetDesc = '>LLiiiiii%dB' % (50 * 300)
            unpacked_data = struct.unpack(packetDesc, data)

            # HEADER
            header = unpacked_data[0]
            # header = int.from_bytes(data[:4], byteorder='big')
            time = unpacked_data[1]
            # time = int.from_bytes(data[4:8], byteorder='big')

            # OCCUPANCY GRID DIMENSIONS
            gridRows = unpacked_data[2]
            # gridRows = int.from_bytes(data[8:12], byteorder='big')
            gridColumns = unpacked_data[3]
            # gridColumns = int.from_bytes(data[12:16], byteorder='big')
            self.rows = gridRows
            self.cols = gridColumns

            # VEHICLE INFORMATION
            carPositionX = unpacked_data[4]
            # carPositionX = int.from_bytes(data[16:20], byteorder='big')
            carPositionY = unpacked_data[5]
            # carPositionY = int.from_bytes(data[20:24], byteorder='big')
            angleOfDecision = unpacked_data[6]
            # angleOfDecision = int.from_bytes(data[24:28], byteorder='big')
            self.car_pos_x = carPositionX
            self.car_pos_y = carPositionY
            self.angle_decision = angleOfDecision

            checkSum = unpacked_data[7]
            # checkSum = int.from_bytes(data[28:32], byteorder='big')

            # OCCUPANCY GRID
            # Define and zero out a grid?
            dataGrid = [[0 for x in range(gridColumns)] for y in range(gridRows)]
            byteOffset = 32
            for i in range(0, gridRows):
                for j in range(0, gridColumns):
                    dataGrid[i][j] = int.from_bytes(data[byteOffset:(byteOffset + 1)], byteorder='big')
                    byteOffset += 1

            # Print the grid
            for k in range(0, gridRows):
                gridRowString = ""
                for l in range(0, gridColumns):
                    gridRowString += (str(dataGrid[k][l])) + " "
                print(gridRowString)

            return dataGrid

        except Exception as e:
            print("EXEC_GUICMD: Parse Error - unable to parse command" + str(e))
            return

gridReader = OccGridPacketReader()


#QtGui.QApplication.setGraphicsSystem('raster')
app = QtGui.QApplication([])
#mw = QtGui.QMainWindow()
#mw.resize(800,800)

win = pg.GraphicsWindow(title="Plotting Occupancy Grid")
win.resize(1000, 600)
win.setWindowTitle('Plotting Occupancy Grid')

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

p6 = win.addPlot(title="Occupancy Grid Detection")

occGridPlot = p6.plot([], [], pen=None, symbol='o')  # setting pen=None disables line drawing
carPosPlot = p6.plot([], [], pen='g', symbol='t1')
decisionPlot = p6.plot([0,0],[0,0], pen='y')


def update_test():
    gridReader.read_test()

def update():
    global occGridPlot
    global carPosPlot
    global decisionPlot

    dataGrid = gridReader.read_grid()

    x = []
    y = []
    for k in range(0, gridReader.rows):
        for l in range(0, gridReader.cols):
            if dataGrid[k][l] == 1:
                x.append(k + 1)
                y.append(l + 1)

    occGridPlot.setData(x, y)
    carPosPlot.setData([gridReader.car_pos_x], [gridReader.car_pos_y])

    # Decision Line
    angle = gridReader.angle_decision
    radius = 25  # Line length
    t_x = math.cos(math.radians(angle)) * radius
    t_y = math.sin(math.radians(angle)) * radius
    # rotate counter 90 deg. clockwise (x,y) -> (-y,x)
    x = t_y
    y = t_x
    line_start_x = int(gridReader.car_pos_x)
    line_start_y = int(gridReader.car_pos_y)
    line_end_x = int(gridReader.car_pos_x + x)
    line_end_y = int(gridReader.car_pos_y + y)
    dec_x = [line_start_x, line_end_x]
    dec_y = [line_start_y, line_end_y]

    decisionPlot.setData(dec_x, dec_y)



# Main Graph Update Loop
timer = QtCore.QTimer()
# timer.timeout.connect(update_test)
timer.timeout.connect(update)
update_test
timer.start(50)


## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()


