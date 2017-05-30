#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Place
class Place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['placed','failed'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state PLACE')
        if self.counter < 3:
            self.counter += 1
            return 'placed'
        else:
            return 'failed'
