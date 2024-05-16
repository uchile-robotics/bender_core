#!/usr/bin/env python
from __future__ import division
import sys
import signal
from PyQt4 import QtGui

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas

import numpy as np

import rospy
from dynamixel_msgs.msg import MotorState, MotorStateList


class Monitor(FigureCanvas):
    def __init__(self, names, y_range=[0.0,1.0]):
        self.fig = Figure()
        self.ax = self.fig.add_subplot(111)


        FigureCanvas.__init__(self, self.fig)
        self.columns = len(names)
        self.data = np.zeros(self.columns)
        self.counter = 1
        # Width of the bars
        self.width = 0.8

        # Locations of the bars
        self.locs = np.arange(self.columns)

        # First set of bars
        self.bars = self.ax.bar(self.locs, self.data, self.width, color=(0.8, 0.0, 0.0))

        # Set up the lables for the bars
        self.ax.set_xticks(self.locs+0.5)
        self.ax.set_xticklabels(names)

        # Set the limit for the x and y
        self.ax.set_xlim(0.0, self.columns)
        self.ax.set_ylim(y_range[0],y_range[1])

        # Draw the canvas
        self.fig.canvas.draw()

    def set_data(self, data):
        for i,bar in enumerate(self.bars):
            bar.set_height(data[i][0])
            temp2red = min(1.0, max(1.1-data[i][1]/MotorMonitor.MAX_TEMP,0.01))
            bar.set_facecolor((0.8, temp2red, temp2red))
        self.fig.canvas.draw()


class MotorMonitor():

    TYPES = {
        'load': [-1.0,1.0],
        'temperature': [0.0, 100.0],
        'error': [-20.0, 20.0]
    }

    MAX_TEMP = 80

    def __init__(self, ids, names, monitor_type='load', rate=10.0):
        # Data
        self.motor_ids = ids
        self.motor_names = names
        self.monitor_type = monitor_type
        self.num = len(self.motor_names)
        self.id_value = dict(zip(self.motor_ids, [[0.0,0.0]]*self.num))

        # Motor monitor GUI
        app = QtGui.QApplication(sys.argv)
        self.w = Monitor(self.motor_names, MotorMonitor.TYPES[self.monitor_type])
        self.w.setWindowTitle('Motor '+ str(self.monitor_type))
        self.w.show()
        # Suscriber
        rospy.loginfo('Suscribing to ' + rospy.get_namespace() + 'motor_states')
        rospy.Subscriber('motor_states',MotorStateList, self.update, queue_size=1)
        # Timer for update canvas
        rospy.Timer(rospy.Duration(1/rate),self.plot)
        # Init QT
        sys.exit(app.exec_())

    def update(self, state_list):
        # Update data from msg
        states = filter(lambda state: state.id in self.motor_ids, state_list.motor_states)
        if len(states)>0:
            for state in states:
                self.id_value[state.id] = [state.__getattribute__(self.monitor_type), state.temperature]
        
    def plot(self, event):
        # Update GUI data
        self.w.set_data(self.id_value.values())

def main():
    signal.signal(signal.SIGINT, sigint_handler)
    rospy.init_node('motor_monitor', disable_signals=True)
    # Get motor_ids
    if not rospy.has_param('~motor_ids'):
        rospy.logerr('No motor_ids for monitor')
        rospy.signal_shutdown('No motor_ids for monitor')
        sys.exit(-1)

    motor_ids = rospy.get_param('~motor_ids')
    # Get motor_names
    motor_names = list()
    if not rospy.has_param('~motor_names'):
        motor_names = ['id: '+str(i) for i in motor_ids]
    else:
        motor_names = rospy.get_param('~motor_names')

    # Check params length
    if (len(motor_ids) != len(motor_names)):
        rospy.logerr('Error in parameters: motor_names and motor_ids must have the same size')
        rospy.signal_shutdown('Error in parameters')
        sys.exit(-1)

    # Get monitor type
    monitor_type = rospy.get_param('~monitor_type','load')
    # Check type
    if not monitor_type in MotorMonitor.TYPES.keys():
        rospy.logerr('Invalid type ' + str(monitor_type))
        rospy.signal_shutdown('Error in parameters')
        sys.exit(-1)
    # Get rate
    rate = rospy.get_param('~rate', 10.0)
    
    # Init monitor
    MotorMonitor(motor_ids, motor_names, monitor_type, rate)

def sigint_handler(*args):
    sys.stderr.write('\r\n')
    QtGui.qApp.closeAllWindows()
    rospy.signal_shutdown('Shutting down motor monitor')
    sys.exit(0)

if __name__ == '__main__':
    main()