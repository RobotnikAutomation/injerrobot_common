#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Inspection
class Inspection(smach.State):
    def __init__(self, sim):
        self._sim = sim
        
        smach.State.__init__(self, outcomes=['good','bad','failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state INSPECTION')
        return 'good'
        #return 'bad'
