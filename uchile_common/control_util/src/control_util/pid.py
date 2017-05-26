# Modified example source code from the book
# "Real-World Instrumentation with Python"
# by J. M. Hughes, published by O'Reilly Media, December 2010,
# ISBN 978-0-596-80956-0.

import rospy
import numpy as np

class CircularBuffer():
    """
    Buffer circular
    """
    def __init__(self, length):
        self.data = np.zeros(length, dtype='f')
        self.index = 0
        self.size = length

    def add(self, x):
        self.data[self.index] = x
        self.index = (self.index + 1) % self.data.size

    def movingaverage(self, window):
        weights = np.repeat(1.0, window)/window
        sma = np.convolve(self.data, weights, 'valid')
        return sma
    
    def mean(self):
        return np.mean(self.data)

    def abs_mean(self):
        return np.mean(np.fabs(self.data))

class PID(object):
    """
    PID control class

    This class implements a simplistic PID control algorithm. When first
    instantiated all the gain variables are set to zero, so calling
    the method compute_output will just return zero.
    """

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, i_clamp=1, error_order=10, max_acc=None):
        # initialize gains
        self._kp = kp
        self._ki = ki
        self._kd = kd

        self._i_clamp = i_clamp

        # initialize error, results, and time descriptors
        self._prev_err = 0.0
        self._cp = 0.0
        self._ci = 0.0
        self._cd = 0.0
        self._cur_time = 0.0
        self._prev_time = 0.0
        self._prev_output = 0.0

        # error moving average for check convergence
        self.error = CircularBuffer(error_order)

        # max_acc to limit output
        self.max_acc = max_acc

        self.initialize()

    def initialize(self):
        """
        Initialize pid controller.
        """
        # reset delta t variables
        self._cur_time = rospy.get_time()
        self._prev_time = self._cur_time

        self._prev_err = 0.0

        # reset result variables
        self._cp = 0.0
        self._ci = 0.0
        self._cd = 0.0

    def set_kp(self, invar):
        """
        Set proportional gain.
        """
        self._kp = invar

    def set_ki(self, invar):
        """
        Set integral gain.
        """
        self._ki = invar

    def set_kd(self, invar):
        """
        Set derivative gain.
        """
        self._kd = invar

    def set_clamp(self, invar):
        """
        Set integral clamp.
        """
        self._i_clamp = invar

    def mean_error(self):
        return self.error.abs_mean()

    def convergence(self, threshold = 0.1):
        return self.error.abs_mean() < threshold

    def compute_output(self, error):
        """
        Performs a PID computation and returns a control value based on
        the elapsed time (dt) and the error signal from a summing junction
        (the error parameter).
        """
        self.error.add(error)

        self._cur_time = rospy.get_time()  # get t
        dt = self._cur_time - self._prev_time  # get delta t
        de = error - self._prev_err  # get delta error

        self._cp = error  # proportional term
        # integral error with iclamp antiwindup scheme
        raw_ci = error * dt
        self._ci += raw_ci if self._i_clamp is not None else max(min(raw_ci, self._i_clamp), -self._i_clamp)

        self._cd = 0
        if dt > 0:  # no div by zero
            self._cd = de / dt  # derivative term

        self._prev_time = self._cur_time  # save t for next pass
        self._prev_err = error  # save t-1 error

        # sum the terms and return the result
        curr_output = ((self._kp * self._cp) + (self._ki * self._ci) +
                    (self._kd * self._cd))

        # check for maximum acceleration limit (if there is one)
        if self.max_acc is not None:
            if curr_output - self._prev_output > self.max_acc:
                curr_output = self._prev_output + self.max_acc
            elif self._prev_output - curr_output > self.max_acc:
                curr_output = self._prev_output - self.max_acc

        # save last output
        self._prev_output = curr_output

        return curr_output
