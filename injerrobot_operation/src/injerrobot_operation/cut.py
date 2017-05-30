#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Cut
class Cut(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cutted','failed'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state CUT')
        if self.counter < 3:
            self.counter += 1
            return 'cutted'
        else:
            return 'failed'
