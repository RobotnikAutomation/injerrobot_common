#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Clip
class Clip(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['clipped','failed'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state CLIP')
        if self.counter < 3:
            self.counter += 1
            return 'clipped'
        else:
            return 'failed'
