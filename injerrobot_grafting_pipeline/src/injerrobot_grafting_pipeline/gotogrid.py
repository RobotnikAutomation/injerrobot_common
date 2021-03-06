#!/usr/bin/env python

import sys

import rospy

import tf
import tf.listener
import tf.transformations

import smach
import smach_ros

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import numpy as np

# define state GoToGrid
class GoToGrid(smach.State):
    def __init__(self, sim):
        smach.State.__init__(self, outcomes=['reached','failed','grid_completed'], input_keys=['params'])
        
        self._sim = sim
        self._current_x = 0
        self._current_y = 0
        
        self._listener = tf.listener.TransformListener()
        
    def execute(self, userdata):
        rospy.loginfo('Executing state GOTOGRID')
        
        rospy.loginfo('Current X: %d, Current Y: %d' % (self._current_x, self._current_y))
        
        if (self.params['grid']['x_first'] == True and self._current_y >= self.params['grid']['Y']) or (self.params['grid']['x_first'] == False and self._current_x >= self.params['grid']['X']):
            return 'grid_completed'
        
        pose = self.params['pose'][0]
        
        
        try:
            self._listener.waitForTransform(self.move_group.getGripperFrame(), self.move_group.getFixedFrame(), rospy.Time(0.), rospy.Duration(4.))
            (trans, rot) = self._listener.lookupTransform(self.move_group.getGripperFrame(), self.move_group.getFixedFrame(), rospy.Time(0.))
            rospy.loginfo("current pose: %f %f %f" % (trans[0], trans[1], trans[2]))
        except tf.Exception:
            rospy.logwarn("couldn't get pose")
        
        
        tf_quat = tf.transformations.quaternion_from_euler(pose[1][0], pose[1][1], pose[1][2])
        
        transformation = np.dot(tf.transformations.translation_matrix(pose[0]), tf.transformations.quaternion_matrix(tf_quat))

        grid_point = geometry_msgs.msg.Point()
        grid_point.x = self._current_x * self.params['grid']['x_step']
        grid_point.y = self._current_y * self.params['grid']['y_step']
        grid_point.z = 0.0
        
        rospy.loginfo("origin: %f %f %f" % (pose[0][0], pose[0][1], pose[0][2]))
        grid_point_transformed = list(np.dot(transformation, np.array([grid_point.x, grid_point.y, grid_point.z, 1.0])))[:3]
        rospy.loginfo("must go to: %f %f %f" % (grid_point_transformed[0], grid_point_transformed[1], grid_point_transformed[2]))
        
        goal = geometry_msgs.msg.PoseStamped()
        goal.header.frame_id = self.move_group.getGripperFrame()
        
        goal.pose.position.x = grid_point_transformed[0]
        goal.pose.position.y = grid_point_transformed[1]
        goal.pose.position.z = grid_point_transformed[2]
        
        
        goal.pose.orientation.x = tf_quat[0]
        goal.pose.orientation.y = tf_quat[1]
        goal.pose.orientation.z = tf_quat[2]
        goal.pose.orientation.w = tf_quat[3]
        
        #self.move_group.moveToPose(goal, 'left_arm_tool0')
        #self.move_group.moveToPose(goal, 'left_arm_link_6')
        self.move_group.moveToPose(goal)
        rospy.loginfo("move to pose")
        rospy.sleep(1.)
        
        try:
            self._listener.waitForTransform(self.move_group.getGripperFrame(), self.move_group.getFixedFrame(), rospy.Time(0.), rospy.Duration(4.))
            (trans, rot) = self._listener.lookupTransform(self.move_group.getGripperFrame(), self.move_group.getFixedFrame(), rospy.Time(0.))
            rospy.loginfo("current pose: %f %f %f" % (trans[0], trans[1], trans[2]))
        except tf.Exception:
            rospy.logwarn("couldn't get pose")        
        
        if self.params['grid']['x_first']:
            self._current_x += 1
            if self._current_x >= self.params['grid']['X']:
                self._current_x = 0
                self._current_y += 1
        else:
            self._current_y += 1
            if self._current_y >= self.params['grid']['Y']:
                self._current_y = 0
                self._current_x += 1
        
        return 'reached'
        
    def resume(self, new_x, new_y):
        self._current_x = new_x
        self._current_y = new_y

    def reset(self):
        self.resume(0,0)
