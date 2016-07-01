#!/usr/bin/env python
# Matias Pavez Bahamondes
# 7/Nov/2014
# last: 22/mar/2015

import rospy
import serial

from bender_head.AbstractHeadInterface import AbstractHeadInterface

# TODO: (locks) para colas que mantienen los mensajes??

class HeadSerialInterface(AbstractHeadInterface):
    ''' Handles serial ports for communication with bender-head. It can reconnect itself if neccessary '''

    def __init__(self, msg_keys):
        
        self.device = None
        
        # for connection testing 
        self.test_connection_msg = chr(255)     # dummy msg --> no debe colisionar con otra direccion ya ocupada!!!!!
        self.last_write_time = rospy.Time.now()
        self.test_time_th = rospy.Duration(1)   # [s]

        # msg handling
        self.msg_keys = msg_keys
        self.curr_msg_dic    = {}
        self.curr_msg_id_dic = {}
        self.last_msg_dic    = {}
        self.last_msg_id_dic = {}
        for key in self.msg_keys:
        
            self.curr_msg_dic[key] = [0]
            self.last_msg_dic[key] = [0]
            self.curr_msg_id_dic[key] = 0
            self.last_msg_id_dic[key] = 0
            
        # instruction*(8) + order // changeFace-> order=0
        # instruction*(8) + order // Speak On -> instruc=0
        # instruction*(8) + order // Speak Off-> instruc=1
        self.emotion_dic = {
            'serious'  : 0*8,
            'happy1'   : 1*8, 'happy2'  : 2*8, 'happy3' : 3*8,
            'sad1'     : 4*8 , 'sad2'   : 5*8, 'sad3'   : 6*8, 
            'angry1'   : 7*8, 'angry2'  : 8*8, 'angry3' : 9*8,
            'surprise' :10*8, 'ashamed' :11*8, 'standby':12*8, 'eyebrow':13*8,
            'greetings':14*8, '1313'    :14*8, 'ear'   :15*8, 'yes'     :16*8,
            'no'     :17*8 , 'lost'    :18*8, 'relaxed':19*8,
            'normal'   :20*8, 'agitated':21*8, 'blink' :22*8, 'flirt'   :23*8
        }
        self.mouth_dic = {'on':0*8+3, 'off':1*8+3}
        

    def close(self):
        # TODO: limpiar colas

        try:    
            if self.device is None:
                rospy.loginfo("Already Disconnected")
                return

            if self.device.isOpen():
                self.device.close()
                rospy.loginfo("Disconnected.")
                return

        except Exception:
            rospy.logerr("A problem ocurred while trying to close the serial port")

    def configure_serial(self):

        # create Serial interface (closed!)
        ser = serial.Serial()
        ser.baudrate = 19200
        ser.port = None           # device name (string), port number (number) or None. 
        ser.timeout = 0           # read timeout (we aren't going to read anything)
        #ser.writeTimeout = None  # Set a write timeout value.
        
        # Read  Timeout: None (wait forever), 0 (non-blocking) or x.x (block x.x[s])
        # Write Timeout: IDEM, ('None' by default)
        # For other configurations see: http://pyserial.sourceforge.net/pyserial_api.html 

        try:
            if self.device is not None and self.device.isOpen():
                self.device.close()
                rospy.loginfo("Disconnected.")

        except Exception:
            rospy.logerr("A problem ocurred while closing the serial communication")

        # set serial device
        self.device = ser

    def is_connected(self):

        try:
            if self.device is None or not self.device.isOpen():
                return False
        except:
            return False

        return True

    def connect(self):
        # TODO: dar hint para el nombre del puerto

        # create device and configuration
        self.configure_serial()

        # try to connect to every port
        for port_no in xrange(0,10):
            
            self.device.port = '/dev/ttyACM' + str(port_no)
            
            try:
                # try to open serial port
                self.device.open()

                # at this point, we found a valid device!
                break

            except Exception:

                # close invalid connection
                self.device.close()
                

        if self.device.isOpen():
            rospy.loginfo('Connected to port: ' + self.device.port)
            return True

        rospy.logerr('Cannot open serial face device')
        self.device = None
        return False

    def write(self, key, value):

        key.lower()
        if key not in self.msg_keys:
            rospy.logwarn('Unkown key: ' + key)
            return

        serial_msg = []
        if key == 'mouth':
            # mouth order
            serial_msg = [self.mouth_dic[value]]
        elif key == 'emotion':
            # emotion order
            serial_msg = [self.emotion_dic[value]]
        else:
            # yaw order
            
            # instruction*(8) + order
            angle = float(value)
            PAN_CENTER = 147
            yaw_order = (int)((-angle - 28)/0.708661 + PAN_CENTER)
            serial_msg = [(0*8+1), yaw_order]

        rospy.loginfo("Setting key: " + key + ", value: " + value + ", with code: '" + str(serial_msg) + "'")

        # encola ordenes a mandar
        self.curr_msg_dic[key]     = serial_msg
        self.curr_msg_id_dic[key] += 1

    def send_orders(self):

        # current time
        curr_time = rospy.Time.now()

        for key in self.msg_keys:

            curr_msg    = self.curr_msg_dic[key]
            curr_msg_id = self.curr_msg_id_dic[key]
            last_msg_id = self.last_msg_id_dic[key]

            if curr_msg_id != last_msg_id:

                #rospy.loginfo("Writing Data... = '" + str(curr_msg) + "'")
                self.last_msg_id_dic[key] = curr_msg_id
                self.last_msg_dic[key]    = curr_msg
                
                # write all orders for this msg
                for index in range(len(curr_msg)):
                    order = curr_msg[index]
                    self.device.write(chr(order))

                self.last_write_time = curr_time

        return

    def loop(self):
        ''' Chequea continuamente el estado de la conexion, intenta reconectar
        de ser necesario y envia los mensages pendientes (p.e, si se intento enviar
        mensajes mientras la cabeza estaba desconectada)'''

        try:
            if self.device is None:

                rospy.logwarn("Reconnecting")
                connection_success = self.connect()

                if not connection_success:
                    rospy.sleep(1)
                    return
            
            # current time
            curr_time = rospy.Time.now()

            # send messages if neccesary
            self.send_orders()

            # send dummy message to test face connection
            if curr_time - self.last_write_time > self.test_time_th:

                # write a dummy msg only with testing purposes
                rospy.logdebug("Writing Dummy Data...")
                self.device.write(self.test_connection_msg)
                self.last_write_time = curr_time
            
        except serial.serialutil.SerialException:
            if(not(self.device is None)):
                self.device.close()
                self.device = None
                rospy.logwarn("Disconnecting")

            rospy.logwarn("No Connection with face serial")
            rospy.sleep(2)
        except Exception as e:
            print str(e)
            pass

        rospy.sleep(1)

