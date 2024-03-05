# Import libraries
import numpy as np
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import serial
import time

SERIAL_PORT = 'COM20'
BAUDRATE = 57600
SERIAL_PORT_READ_TIMEOUT_MS  = 5000

def openSerial(serialPort=SERIAL_PORT):
    return serial.Serial(port=serialPort,baudrate=BAUDRATE,timeout=SERIAL_PORT_READ_TIMEOUT_MS)

def getData_PSU(sp,arg):
    if arg == 'B':
        sp.write('aget0\r')
        while (sp.inWaiting()==0):
            pass
        return int(sp.readline().split('A\r\n')[0])*(5.03/1024)*(float(301+150)/(150))
    elif arg == 'A':
        sp.write('aget1\r')
        while (sp.inWaiting()==0):
            pass
        return sp.readline().split('V\r\n')[0]
    elif arg == 'U':
        sp.write('aget2\r')
        while (sp.inWaiting()==0):
            pass
        return sp.readline().split('W\r\n')[0]
    elif arg == 'Z':
        sp.write('get3\r')
        while (sp.inWaiting()==0):
            pass
        return sp.readline().split('C\r\n')[0]
    elif arg == 'L':
        sp.write('get8\r')
        while (sp.inWaiting()==0):
            pass
        return sp.readline().split('\r\n')[0]

INIT_TEXT = 'DDAT_INITIALISED\r\n'
INIT_TEXT_LENGTH = len(INIT_TEXT)

# Create object serial port
portName = SERIAL_PORT                    # replace this port name by yours!
baudrate = BAUDRATE

### START QtApp #####
app = QtGui.QApplication([])            # you MUST do this once (initialize things)
####################

win = pg.GraphicsWindow(title="Signal from serial port") # creates a window
p = win.addPlot(title="Realtime plot")  # creates empty space for the plot in the window
curve = p.plot(pen='r')                     # create an empty "plot" (a curve to plot)

windowWidth = 500                       # width of the window displaying the curve
Xm = np.linspace(0,0,windowWidth)          # create array that will contain the relevant time series     
ptr = -windowWidth                      # set first x position

curve2 = p.plot(pen='b')
Xm2 = np.linspace(0,0,windowWidth)
ptr2 = -windowWidth

# Realtime data plot. Each time this function is called, the data display is updated
# update current(Ampere) plot
def update_plot_data():
    global curve, ptr, Xm    
    Xm[:-1] = Xm[1:]                      # shift data in the temporal mean 1 sample left
    value = getData_PSU(sp,'B')           # read line (single value) from the serial port
    Xm[-1] = float(value)                 # vector containing the instantaneous values      
    ptr += 1                              # update x position for displaying the curve
    curve.setData(Xm)                     # set the curve with this data
    curve.setPos(ptr,0)                   # set x position in the graph to 0
    QtGui.QApplication.processEvents()    # you MUST process the plot now

# udpate voltage plot
def update_plot2_data():
    global curve2, ptr2, Xm2    
    Xm2[:-1] = Xm2[1:]                      # shift data in the temporal mean 1 sample left
    value = getData_PSU(sp,'Z')               # read line (single value) from the serial port
    Xm2[-1] = float(value)                 # vector containing the instantaneous values      
    ptr2 += 1                              # update x position for displaying the curve
    curve2.setData(Xm2)                     # set the curve with this data
    curve2.setPos(ptr2,0)                   # set x position in the graph to 0
    QtGui.QApplication.processEvents()    # you MUST process the plot now


if __name__ == "__main__":
    sp = openSerial()
    timeStamp = time.time()
    while time.time()-timeStamp < SERIAL_PORT_READ_TIMEOUT_MS/1000 and sp.inWaiting() < INIT_TEXT_LENGTH:
        pass

    sp.read(sp.inWaiting())#flush buffer

    # sp.write('fanf 1\r')
    # sp.readline()
    # sp.write('fanp 140\r')
    # sp.readline()
    sp.write('seth7\r')
    sp.readline()
    time.sleep(1)
    sp.read(sp.inWaiting())#flush buffer


    ### MAIN PROGRAM #####    
    # this is a brutal infinite loop calling your realtime data plot
    while True: 
        update_plot_data()
        update_plot2_data()

    ### END QtApp ####
    pg.QtGui.QApplication.exec_() # you MUST put this at the end
    ##################