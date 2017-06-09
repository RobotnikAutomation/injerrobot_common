#!/usr/bin/env python

import rospy
import robotnik_msgs.msg 
import robotnik_msgs.srv

class IoModule:
    
    def __init__(self, sim):
        self._sim = sim
        
        self.io_service = '~set_io'
        self.io_topic = 'io'
        
        self._current_io = robotnik_msgs.msg.inputs_outputs()
        
        if self._sim == True:
            # if we are in simulation mode, advertise ourselves the service and the topic
            self.io_srv_handle = rospy.Service(self.io_service, robotnik_msgs.srv.set_digital_output, self.io_srv_callback)
            self.io_pub = rospy.Publisher(self.io_topic, robotnik_msgs.msg.inputs_outputs, queue_size=1)
            
            n = 16
            self._current_io.digital_inputs =  [0] * n
            self._current_io.digital_outputs = [0] * n
            self._current_io.analog_inputs =   [0] * n
            self._current_io.analog_outputs =  [0] * n
        
        rospy.wait_for_service(self.io_service)
        self.io_srv = rospy.ServiceProxy(self.io_service, robotnik_msgs.srv.set_digital_output)
        
        rospy.Subscriber(self.io_topic, robotnik_msgs.msg.inputs_outputs, self.io_callback, queue_size = 1)
    
    def io_srv_callback(self, req):
        if req.output < 0 or req.output > len(self._current_io.digital_outputs):
            return robotnik_msgs.srv.set_digital_outputResponse(False)

        rospy.loginfo('set %d to %r' % (req.output, req.value))
        self._current_io.digital_outputs[req.output] = req.value
        return robotnik_msgs.srv.set_digital_outputResponse(True)

    def io_callback(self, msg):
        self._current_io = msg

    ### XXX: check number is 0 or 1 indexed??
    def set_output(self, number, value):
        srv_result = self.io_srv(number, value)
        
        if self._sim == True:
            self.io_pub.publish(self._current_io)
            
        return srv_result
        
    
    def set_input(self, number, value):
        if self._sim == False: # it is nonsense to set an input in the real world...
            return False
            
        if number < 0 or number > len(self._current_io.digital_inputs):
            return False
            
        self._current_io.digital_inputs[number] = value
        self.io_pub.publish(self._current_io)
    
    def get_output(self, number):
        if number < 0 or number > len(self._current_io.digital_inputs):
            return -1
        return self._current_io.digital_outputs[number]
    
    def get_input(self, number):
        if number < 0 or number > len(self._current_io.digital_inputs):
            return -1
        return self._current_io.digital_inputs[number]
