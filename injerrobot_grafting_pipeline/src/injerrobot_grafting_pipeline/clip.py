#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Clip
class Clip(smach.State):
    def __init__(self, sim):
        self._sim = sim
        
        smach.State.__init__(self, outcomes=['clipped','failed'], input_keys=['params'])
        
        self._wait_for_clipper = 1.0
        
    def execute(self, userdata):
        rospy.loginfo('Executing state CLIP')

        if self._sim == True:
            self.io_module.set_input(userdata.params['input']['clipper'], True)
            rospy.sleep(self._wait_for_clipper)


        if self.io_module.get_input(userdata.params['input']['clipper']) != True:
            rospy.logerr('CLIP: there is not plant. Failed')
            return 'failed'

        rospy.loginfo('CLIP: plant detected, putting clip')
        self.io_module.set_output(userdata.params['output']['clipper'], True)
        
        rospy.sleep(self._wait_for_clipper)
        
        rospy.loginfo('CLIP: Putted clip')

        return 'clipped'
