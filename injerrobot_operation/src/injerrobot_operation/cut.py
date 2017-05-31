#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Cut
class Cut(smach.State):
    def __init__(self, sim):
        self._sim = sim
        
        smach.State.__init__(self, outcomes=['cutted','failed'], input_keys=['params'])
        
        self._wait_for_cutter = 1.0
        
    def execute(self, userdata):
        rospy.loginfo('Executing state CUT')

        if self._sim == True:
            self.io_module.set_input(userdata.params['input']['cutter'], True)
            rospy.sleep(self._wait_for_cutter)


        if self.io_module.get_input(userdata.params['input']['cutter']) != True:
            rospy.logerr('CUT: there is not plant. Failed')
            return 'failed'

        rospy.loginfo('CUT: plant detected, closing gripper')
        self.io_module.set_output(userdata.params['output']['cutter'], True)
        
        rospy.sleep(self._wait_for_cutter)
        
        rospy.loginfo('CUT: Closed gripper')

        return 'cutted'
