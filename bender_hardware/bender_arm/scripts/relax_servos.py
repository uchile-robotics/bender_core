#!/usr/bin/env python

"""
  Disabling and enabling the torque for servos.
"""

import rospy
from bender_srvs.srv import Bool, BoolResponse
from dynamixel_controllers.srv import TorqueEnable

class Relax():
    def __init__(self):
        rospy.init_node('relax_servos')

        start_disable = rospy.get_param('~start_disable', True)
        
        self.controllers = rospy.get_param('~controllers', '')
        rospy.loginfo('Control toque enable for ' + str(self.controllers))
        self.torque_services = list()

        rospy.Service('~torque_enable', Bool, self.process_req)

        for controller in sorted(self.controllers):
            # Torque service name
            torque_service = controller + '/torque_enable'
            rospy.loginfo('Waiting for {0}{1}'.format(rospy.get_namespace(),torque_service))
            rospy.wait_for_service(torque_service)
            self.torque_services.append(rospy.ServiceProxy(torque_service, TorqueEnable))

        # Deshabilitar torques al inicio
        if start_disable:
            rospy.sleep(2.0)
            self.send_req()

    def process_req(self, req):
        if req.data:
            return BoolResponse(self.send_req( True, 'enabling'))
        else:
            return BoolResponse(self.send_req())

    def send_req(self, req = False, msg = 'disabling'):
        succeed = True
        for i, service in enumerate(self.torque_services):
            joint_name = self.controllers[i].replace('_controller', '') + '_joint'
            try:
                service(req)
                rospy.loginfo('Succeed ' + msg + ' torque for ' + joint_name)
            except:
                rospy.logwarn('Fail ' + msg + ' torque for ' + joint_name)
                succeed = False
        return succeed
    
if __name__=='__main__':
    Relax()
    rospy.spin()
    