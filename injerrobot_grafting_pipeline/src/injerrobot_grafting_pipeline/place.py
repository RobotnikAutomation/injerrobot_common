#!/usr/bin/env python

import rospy
import smach
import smach_ros

import geometry_msgs.msg

import baxter_interface

import moveit_commander
import tf
import tf.transformations

# define state Place
class Place(smach.State):
    def __init__(self, sim):
        self._sim = sim
        
        smach.State.__init__(self, outcomes=['placed','failed'], input_keys=['params'])
        self._gripper = baxter_interface.Gripper("right")
        self._wait_for_gripper = 1.0
        self.max_velocity_scaling_factor = 1.0
    def execute(self, userdata):
        rospy.loginfo('Executing state PLACE')
        
        
        self._gripper.close()
        rospy.logwarn("SLEEP GRIPPER")
        rospy.sleep(self._wait_for_gripper)
        
        if 'preplace' in self.params.keys():
            rospy.loginfo('PLACE: executing PREPLACE CARTESIAN MOVEMENT.')
            for pose in self.params['preplace']:
                goal = geometry_msgs.msg.Pose()
            
                goal.position.x = pose[0][0]
                goal.position.y = pose[0][1]
                goal.position.z = pose[0][2]
                
                tf_quat = tf.transformations.quaternion_from_euler(pose[1][0], pose[1][1], pose[1][2])
                goal.orientation.x = tf_quat[0]
                goal.orientation.y = tf_quat[1]
                goal.orientation.z = tf_quat[2]
                goal.orientation.w = tf_quat[3]
                
                self.move_group.moveToPoseCartesianPathCommander(goal, max_velocity_scaling_factor=self.max_velocity_scaling_factor)
        
        
        self._gripper.open()
        rospy.logwarn("SLEEP GRIPPER")
        rospy.sleep(self._wait_for_gripper)
        
        if 'postplace' in self.params.keys():
            rospy.loginfo('PLACE: executing POSTPLACE CARTESIAN MOVEMENT.')
            for pose in self.params['postplace']:
                goal = geometry_msgs.msg.Pose()
                
                goal.position.x = pose[0][0]
                goal.position.y = pose[0][1]
                goal.position.z = pose[0][2]
                
                tf_quat = tf.transformations.quaternion_from_euler(pose[1][0], pose[1][1], pose[1][2])
                goal.orientation.x = tf_quat[0]
                goal.orientation.y = tf_quat[1]
                goal.orientation.z = tf_quat[2]
                goal.orientation.w = tf_quat[3]
                
                self.move_group.moveToPoseCartesianPathCommander(goal, max_velocity_scaling_factor=self.max_velocity_scaling_factor)
        
        
        return 'placed'
