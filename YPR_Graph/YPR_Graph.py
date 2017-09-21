'''
Created on Aug 15, 2017

@author: Murph
'''

# Import libraries

import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore

#from pyqtgraph.Qt import QtGui, QtCore
import serial
from numpy import *
import logging

#serial port creation
#portName = '/dev/ttyACM0' #Linux
portName = "COM5" # Windows - change to reflect your comport
baudrate = 115200
ser = serial.Serial(portName,baudrate)

### START QtApp ######
app = QtGui.QApplication([]) 
###########################
pg.setConfigOption('background', 'b')
pg.setConfigOption('foreground', 'y')
win = pg.GraphicsWindow(title="Signal from IMU") # creates a window
p = win.addPlot(title="IMU Yaw Data", row=1, col=0)  # creates empty space for the plot in the window
y = win.addPlot(title="IMU Roll/Pitch Data", row =2, col=0)
p.addLegend()
y.addLegend()
p.setYRange(-10,370, padding = 0)
y.setYRange(-190, 190, padding = 0)
p.setLabel('left', "Degrees")
y.setLabel('left', "Degrees")
p.setLabel('bottom', 'Time')
y.setLabel('bottom', 'Time')
p.showGrid(True, True, 0.8)
y.showGrid(True, True, 0.8)
roll = y.plot(pen ='r', name = 'Roll')                        # create an empty "plot" (a curve to plot)
pitch = y.plot(pen ='g', name = 'Pitch')
yaw = p.plot(pen = 'w', name = 'Yaw')
windowWidth = 200                       # width of the window displaying the curve
Xm = linspace(0,0,windowWidth)          # create array that will contain the relevant time series 
Xm1 = linspace(0,0,windowWidth)
Xm2 = linspace(0,0,windowWidth)
ptr = -windowWidth                      # set first x position
ser.write(b'#o1')

# Realtime data plot. Each time this function is called, the data display is updated

def update():
	global curve, ptr, Xm, Xm1, Xm2	
	while True:
		Xm[:-1] = Xm[1:]  
		Xm1[:-1] = Xm1[1:]
		Xm2[:-1] = Xm2[1:]
		while (ser.inWaiting()==0):
		   pass # shift data in the temporal mean 1 sample left
		value = ser.readline().strip(b'\r\n')
		value = value.split(b',')
		line_len = len(value)
		if value[0] == b'#' and line_len == 4:
			Xm[-1] = float(value[1])
			Xm1[-1] = float(value[2])
			Xm2[-1] = float(value[3])
                         # update x position for displaying the curve
		ptr += 1
		roll.setData(Xm)                     # set the curve with this data
		pitch.setData(Xm1)
		yaw.setData(Xm2)
		roll.setPos(ptr,0)                   # set x position in the graph to 0
		pitch.setPos(ptr,0)
		yaw.setPos(ptr,0)
		QtGui.QApplication.processEvents()    # you MUST process the plot now

### MAIN PROGRAM #####    
# this is a brutal infinite loop calling your realtime data plot
while True: update()

### END QtApp ####
pg.QtGui.QApplication.exec_() # you MUST put this at the end
##################
