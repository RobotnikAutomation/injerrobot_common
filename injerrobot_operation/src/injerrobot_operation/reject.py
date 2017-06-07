#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Reject
class Reject(smach.State):
    def __init__(self, sim):
        self._sim = sim
        
        smach.State.__init__(self, outcomes=['rejected','failed'], input_keys=['params'])
        
        self._wait_for_clipper = 1.0
        
    def execute(self, userdata):
        rospy.loginfo('Executing state REJECT')

        if self._sim == True:
            self.io_module.set_input(userdata.params['input']['gripper'], True)
            rospy.sleep(self._wait_for_clipper)

        self.io_module.set_output(userdata.params['output']['gripper'], True)
        
        rospy.sleep(self._wait_for_clipper)
        
        rospy.loginfo('REJECT: Rejected plant')

        return 'rejected'
