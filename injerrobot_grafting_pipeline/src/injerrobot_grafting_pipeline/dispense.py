#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Dispense
class Dispense(smach.State):
    def __init__(self, sim):
        self._sim = sim
        
        smach.State.__init__(self, outcomes=['dispensed','failed'], input_keys=['params'])
        
        self._wait_for_gripper = 1.0
        
    def execute(self, userdata):
        rospy.loginfo('Executing state DISPENSE')

        if self._sim == True:
            self.io_module.set_input(userdata.params['input']['gripper'], True)
            rospy.sleep(self._wait_for_gripper)


        if self.io_module.get_input(userdata.params['input']['gripper']) != True:
            rospy.logerr('DISPENSE: there is not plant. Failed')
            return 'failed'

        rospy.loginfo('DISPENSE: plant detected, dispensing')
        self.io_module.set_output(userdata.params['output']['gripper'], True)
        
        rospy.sleep(self._wait_for_gripper)
        
        rospy.loginfo('DISPENSE: Plant dispensed')

        return 'dispensed'
