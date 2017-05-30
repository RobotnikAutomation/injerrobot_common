#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Inspect
class Inspection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['good','bad'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state INSPECT')
        if self.counter < 3:
            self.counter += 1
            return 'good'
        else:
            return 'bad'
