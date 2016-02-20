#!/usr/bin/env python
# Matias Pavez Bahamondes
# 22/Marzo/2015


import rospy
import httplib

# TODO: (locks) para colas que mantienen los mensajes??

class HeadHTTPInterface():
    ''' Handles HTTP-GET requests for communication with bender-head. '''

    def __init__(self, msg_keys):
        
        # config
        self.head_hostname = "bender-head"
        self.head_ip = "192.168.0.40"
        self.get_form = "control.html"
                
        # preparation       
        self.uri_server = self.head_ip
        self.uri_query  = "/" + self.get_form
            
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

    def close(self):
        # TODO: limpiar colas
        # TODO: check conn. state and close connection
        # conn.close()
        rospy.loginfo("Disconnected.")
        return

    def is_connected(self):
        # do a connectivity test
        try:
            conn = httplib.HTTPConnection(self.uri_server,timeout=1)
            conn.request("GET", self.uri_query)
            r1 = conn.getresponse()
            #rospy.loginfo("status: " + str(r1.status) + ", reason: " + r1.reason)
            #rospy.loginfo("read: " + r1.read())
            
            if r1.status == 200:
                rospy.loginfo("HTTP-GET connectivity test succeeded for server: " + self.uri_server + self.uri_query)
                conn.close()
                return True

            
        except Exception as e:
            rospy.logwarn("HTTP-GET connectivity test failed for server: " + self.uri_server 
                        + self.uri_query + ". Exception: " + str(e) )
            conn.close()
            return False
        
        rospy.logwarn("HTTP-GET connectivity test failed with code: " + str(r1.status)
                     + ", for uri: " + self.uri_server)
        return False
        

    def connect(self):
        # Cannot ensure a valid connection over time
        # prefer 'is_connected' method
        return True

    def write(self, key, value):
        
        if key not in self.msg_keys:
            rospy.logwarn('Unkown key: ' + key)
            return
        
        # encola ordenes a mandar
        self.curr_msg_dic[key]     = value
        self.curr_msg_id_dic[key] += 1
        
    def send_orders(self):
        
        # current time
        curr_time = rospy.Time.now()
        
        query = ""
        to_send_keys = []
        first_arg = True
        for key in self.msg_keys:

            curr_msg    = self.curr_msg_dic[key]
            curr_msg_id = self.curr_msg_id_dic[key]
            last_msg_id = self.last_msg_id_dic[key]

            if curr_msg_id != last_msg_id:
                
                # append key
                to_send_keys.append(key)
                
                # build uri
                if first_arg:
                    query = query + key + "=" + curr_msg
                    first_arg = False
                else:
                    query = query + "&" + key + "=" + curr_msg
                
                #
            #
        
        # Empty Query
        if not to_send_keys:
            return
            
        # try to send commands
        #rospy.loginfo("Writing Data... = '" + str(curr_msg) + "'")
        succeeded = False
        try:
            conn = httplib.HTTPConnection(self.uri_server,timeout=1)
            conn.request("GET", self.uri_query + "?" + query)
            r1 = conn.getresponse()
            #rospy.loginfo("status: " + str(r1.status) + ", reason: " + r1.reason)
            #rospy.loginfo("read: " + r1.read())
            
            if r1.status == 200:
                rospy.loginfo("HTTP-GET: setting parameters OK for query:" + query)
                succeeded = True
            else:
                rospy.logwarn("Failed to send GET request with HTTP code: "
                         + str(r1.status) + ", for query: " + self.uri_query + "?" + query)
            
        except Exception as e:
            rospy.logwarn("Failed to send HTTP-GET request to server. Query: "
                 + query + ". Exception: " + str(e) )
        
        # failed
        if not succeeded:
            return
        
        # succeeded: clean msg list
        for key in to_send_keys:
            self.last_msg_id_dic[key] = self.curr_msg_id_dic[key]
            self.last_msg_dic[key]    = self.curr_msg_dic[key]

        self.last_write_time = curr_time
        return

    def loop(self):
        ''' envia los mensages pendientes (p.e, si se intento enviar
        mensajes mientras la cabeza estaba desconectada)'''
        
        # send messages if neccesary
        self.send_orders()
        rospy.sleep(0.5)

