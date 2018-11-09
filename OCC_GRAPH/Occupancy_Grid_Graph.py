# -*- coding: utf-8 -*-
"""
This example demonstrates many of the 2D plotting capabilities
in pyqtgraph. All of the plots may be panned/scaled by dragging with
the left/right mouse buttons. Right click on any plot to show a context menu.
"""
HOST = '192.168.4.30'
# HOST = '10.2.127.123'
# HOST = '198.162.4.40'
UDP_PORT = 12346  # The same port as used by the server
# import initExample ## Add path to library (just for examples; you do not need this)

connection_socket = 0

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

            ################################################################
            # OCCUPANCY GRID UDP RECEIVE
            ################################################################
            # header = 0x55555555
            # time = datetime.datetime.utcnow()
            # gridRows = 100
            # gridColumns = 160
            # carPositionX = 25
            # carPositionY = 0
            # angleOfDecision = +- 45
            # gridArray = *
            trows = 100
            tcols = 160

            data, addr = connection_socket.recvfrom(30000)
            packetDesc = '>LLLLiii%dB' % (trows * tcols)
            unpacked_data = struct.unpack(packetDesc, data)

            # HEADER
            header = unpacked_data[0]
            time = unpacked_data[1]

            # OCCUPANCY GRID DIMENSIONS
            gridRows = unpacked_data[2]
            gridColumns = unpacked_data[3]
            self.rows = gridRows
            self.cols = gridColumns

            # VEHICLE INFORMATION
            carPositionX = unpacked_data[4]
            carPositionY = unpacked_data[5]
            angleOfDecision = unpacked_data[6]
            self.car_pos_x = 80 # carPositionX
            self.car_pos_y = 0 # carPositionY
            self.angle_decision = angleOfDecision

            # OCCUPANCY GRID
            unpack_index = 7
            dataGrid = [[0 for x in range(gridColumns)] for y in range(gridRows)]
            for i in range(0, gridRows):
                for j in range(0, gridColumns):
                    dataGrid[i][j] = unpacked_data[unpack_index]
                    unpack_index = unpack_index + 1

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

p6 = win.addPlot(row=100, col=160, title="Occupancy Grid")
p6.setXRange(0,160)
p6.setYRange(0,100)

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
                x.append(l + 1)
                y.append(k + 1)

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


connection_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
connection_socket.bind((HOST, UDP_PORT))
print('connected %s', HOST)

timer.timeout.connect(update)
update_test
timer.start(50)


## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()


