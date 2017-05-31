#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Place
class Place(smach.State):
    def __init__(self, sim):
        self._sim = sim
        
        smach.State.__init__(self, outcomes=['placed','failed'], input_keys=['params'])
        

    def execute(self, userdata):
        rospy.loginfo('Executing state PLACE')
        return 'placed'
