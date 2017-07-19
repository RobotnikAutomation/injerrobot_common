#!/usr/bin/env python

import rospy
import smach
import smach_ros

import geometry_msgs.msg

import baxter_interface

import moveit_commander
import tf
import tf.transformations

# define state Clip
class Clip(smach.State):
    def __init__(self, sim):
        self._sim = sim
        
        smach.State.__init__(self, outcomes=['clipped','failed'], input_keys=['params'])
        
        self._wait_for_clipper = 1.0
        
    def execute(self, userdata):
        rospy.loginfo('Executing state CLIP')

        #~ if self._sim == True:
            #~ self.io_module.set_input(userdata.params['input']['clipper'], True)
            #~ rospy.sleep(self._wait_for_clipper)


        #~ if self.io_module.get_input(userdata.params['input']['clipper']) != True:
            #~ rospy.logerr('CLIP: there is not plant. Failed')
            #~ return 'failed'

        if 'preclip' in self.params.keys():
            rospy.loginfo('CLIP: executing PRECLIP CARTESIAN MOVEMENT.')
            for pose in self.params['preclip']:
                goal = geometry_msgs.msg.Pose()
            
                goal.position.x = pose[0][0]
                goal.position.y = pose[0][1]
                goal.position.z = pose[0][2]
                
                tf_quat = tf.transformations.quaternion_from_euler(pose[1][0], pose[1][1], pose[1][2])
                goal.orientation.x = tf_quat[0]
                goal.orientation.y = tf_quat[1]
                goal.orientation.z = tf_quat[2]
                goal.orientation.w = tf_quat[3]
                
                self.move_group.moveToPoseCartesianPathCommander(goal, max_velocity_scaling_factor=0.01, wait=True)


        rospy.loginfo('CLIP: plant detected, putting clip')
        self.io_module.set_output(userdata.params['output']['clipper'], True)
        
        rospy.sleep(self._wait_for_clipper)
        
        rospy.loginfo('CLIP: Putted clip')


        if 'postclip' in self.params.keys():
            rospy.loginfo('CLIP: executing POSTCLIP CARTESIAN MOVEMENT.')
            for pose in self.params['postclip']:
                goal = geometry_msgs.msg.Pose()
                
                goal.position.x = pose[0][0]
                goal.position.y = pose[0][1]
                goal.position.z = pose[0][2]
                
                tf_quat = tf.transformations.quaternion_from_euler(pose[1][0], pose[1][1], pose[1][2])
                goal.orientation.x = tf_quat[0]
                goal.orientation.y = tf_quat[1]
                goal.orientation.z = tf_quat[2]
                goal.orientation.w = tf_quat[3]
                
                self.move_group.moveToPoseCartesianPathCommander(goal, max_velocity_scaling_factor=0.01)




        return 'clipped'
