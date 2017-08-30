#!/usr/bin/python

import os
import inspect

import rospy
import rospkg
import sensor_msgs.msg
import tf.listener
import tf.transformations

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QDialog, QFileDialog, QMessageBox, QTableWidgetItem
from python_qt_binding.QtCore import Qt

import yaml

class RqtGoto(QDialog):

    def __init__(self):
        super(RqtGoto, self).__init__()
        # Give QObjects reasonable names
        self.setObjectName('RqtGoto')

        
        # QDialog has been already created
        ### Create QWidget
        ### self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_injerrobot'), 'resource', 'RqtGoToDialog.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self) # load to dialog instead of self._widget
        # Give QObjects reasonable names
        #self._widget.setObjectName('RqtInjerrobotUi')
        
        # add signals/slots
        #self._widget.selectYamlPushButton.pressed.connect(self.press_select_yaml)
        #self._widget.loadYamlPushButton.pressed.connect(self.press_load_yaml)
        #self._widget.saveYamlPushButton.pressed.connect(self.press_save_yaml)
        
        #self.gridTable.currentItemChanged.connect(self.grid_table_changed)
        self.gridTable.itemChanged.connect(self.grid_item_activated)
        
        self.addPoseButton.pressed.connect(self.press_add_pose)
        self.addJointPoseButton.pressed.connect(self.press_add_joint_pose)
        
        self.deletePoseButton.pressed.connect(self.press_delete_pose)
        self.deleteJointPoseButton.pressed.connect(self.press_delete_joint_pose)
        
        self.jointPosesBox.currentIndexChanged.connect(self.joint_pose_selected)
        self.posesBox.currentIndexChanged.connect(self.pose_selected)
        self.gridCheckBox.stateChanged.connect(self.grid_checkbox_changed)
       
        self.step = None
        self.step_name = None
        self.joint_state = sensor_msgs.msg.JointState()
        self.base_link = None
        self.end_effector = None
        
        self._received_joint_states = False
        self._name = "RqtGoto"
        
        self._jointstate_sub = rospy.Subscriber('joint_states', sensor_msgs.msg.JointState, self.jointstate_cb)
        self._listener = tf.listener.TransformListener()

    def exec_(self):
        self.set_current_step()
        return super(RqtGoto, self).exec_()
        
    def set_joint_names(self, names):
        self.joint_state.name = names
        self.joint_state.position = [None] * len(names)
        self._received_joint_states = False
# model

# controller
    def add_joint_pose(self, joint_pose):
        try:
            if 'joints' not in self.step.keys():
                self.step['joints'] = []
            self.step['joints'].append(joint_pose)
        except Exception, e:
            rospy.logerr('%s::%s: : cannot add joint pose for %s. %s' % (self._name, self.get_function_name(), self.step_name, e))
    

    def add_pose(self, pose):
        try:
            if 'poses' not in self.step.keys():
                self.step['poses'] = []
            self.step['poses'].append(pose)
            print self.step['poses']
        except Exception, e:
            rospy.logerr('%s::%s: : cannot add pose for %s. %s' % (self._name, self.get_function_name(), self.step_name, e))
        
        
    def delete_joint_pose(self, index):
        rospy.loginfo('%s::%s:' % (self._name, self.get_function_name()))
        joint_poses = self.get_joint_poses()
        if index >= len(joint_poses) or index < 0:
            return
        del joint_poses[index]
        self.step['joints'] = joint_poses

    def get_joint_poses(self):
        rospy.loginfo('%s::%s:' % (self._name, self.get_function_name()))
        joint_poses = []
        try:
            joint_poses = self.step['joints']
        except Exception, e:
            rospy.logerr('%s::%s: : cannot get joint poses for %s. %s' % (self._name, self.get_function_name(), self.step_name, e))
        return joint_poses

    def get_joint_pose(self, index):
        rospy.loginfo('%s::%s:' % (self._name, self.get_function_name()))
        
        if index < 0:
            return None
            
        joint_pose = None
        try:
            joint_pose = self.step['joints'][index]
        except Exception, e:
            rospy.logerr('%s::%s: : cannot get pose for %s. %s' % (self._name, self.get_function_name(), self.step_name, e))
        return joint_pose

    def get_poses(self):
        rospy.loginfo('%s::%s:' % (self._name, self.get_function_name()))
        poses = []
        try:
            poses = self.step['poses']
        except Exception, e:
            rospy.logerr('%s::%s: : cannot get poses for %s. %s' % (self._name, self.get_function_name(), self.step_name, e))
        return poses
 
    def get_pose(self, index):
        rospy.loginfo('%s::%s:' % (self._name, self.get_function_name()))

        if index < 0:
            return None
            
        pose = None
        try:
            pose = self.step['poses'][index]
        except Exception, e:
            rospy.logerr('%s::%s: : cannot get pose for %s. %s' % (self._name, self.get_function_name(), self.step_name, e))
        return pose
 
    def delete_pose(self, index):
        rospy.loginfo('%s::%s:' % (self._name, self.get_function_name()))
        poses = self.get_poses()
        if index >= len(poses) or index < 0:
            return
        del poses[index]
        self.step['poses'] = poses
    
    def add_grid(self):
        self.step['grid'] = dict()
        self.step['grid']['X'] = 1
        self.step['grid']['Y'] = 1
        self.step['grid']['x_first'] = True
        self.step['grid']['x_step'] = 0
        self.step['grid']['y_step'] = 0
    
    def remove_grid(self):
        self.step.pop('grid', None)
        
    def set_current_grid(self):
        
        if 'grid' not in self.step.keys():
            self.add_grid()
            
        self.gridTable.setItem(0, 1, QTableWidgetItem(self.step['grid']['X']))
        self.gridTable.setItem(1, 1, QTableWidgetItem(self.step['grid']['Y']))
        self.gridTable.setItem(2, 1, QTableWidgetItem(self.step['grid']['x_first']))
        self.gridTable.setItem(3, 1, QTableWidgetItem(self.step['grid']['x_step']))
        self.gridTable.setItem(4, 1, QTableWidgetItem(self.step['grid']['y_step']))
        
    def set_current_step(self):
        rospy.loginfo('%s::%s:' % (self._name, self.get_function_name()))
        if self.step == None:
            return
        
        rospy.loginfo('%s::%s: before clear' % (self._name, self.get_function_name()))
        self.jointPosesBox.clear()
        rospy.loginfo('%s::%s: after clear' % (self._name, self.get_function_name()))
        for i in range(len(self.get_joint_poses())):
            rospy.loginfo('%s::%s: in for' % (self._name, self.get_function_name()))
            self.jointPosesBox.addItem(str(i))
        self.jointPosesBox.setCurrentIndex(0)

        self.posesBox.clear()
        for i in range(len(self.get_poses())):
            self.posesBox.addItem(str(i))
        self.posesBox.setCurrentIndex(0)
        
        if 'grid' in self.step.keys():
            self.gridCheckBox.setCheckState(Qt.Checked)
            self.set_current_grid()
        else:
            self.gridCheckBox.setCheckState(Qt.Unchecked)
    

# view
    def joint_pose_selected(self, index):
        rospy.loginfo('%s::%s: index: %d' % (self._name, self.get_function_name(), index)) 
        
        if index < 0:
            self.jointPosesTable.setRowCount(0)
            
        joint_pose = self.get_joint_pose(index)

        if joint_pose == None or len(joint_pose) != len(self.joint_state.name):
            return
            
        self.jointPosesTable.setRowCount(0)
        for i in range(len(joint_pose)):
            row_count = self.jointPosesTable.rowCount()
            self.jointPosesTable.insertRow(row_count)
            self.jointPosesTable.setItem(row_count, 0, QTableWidgetItem(self.joint_state.name[i]))
            self.jointPosesTable.setItem(row_count, 1, QTableWidgetItem(str(joint_pose[i])))
            rospy.loginfo('%s::%s: adding : %s, %f' % (self._name, self.get_function_name(), self.joint_state.name[i], joint_pose[i])) 
    
    def pose_selected(self, index):
        rospy.loginfo('%s::%s:' % (self._name, self.get_function_name()))
        
        if index < 0:
            self.posesTable.setRowCount(0)
            
        pose = self.get_pose(index)

        if pose == None or len(pose) != 2 or len(pose[0]) != 3 or len(pose[1]) != 3:
            return
            
        axes = [['X', 'Y', 'Z'],['roll', 'pitch', 'yaw']]
        self.posesTable.setRowCount(0)
        for i in range(len(axes)):
            for j in range(len(axes[i])):
                row_count = self.posesTable.rowCount()
                self.posesTable.insertRow(row_count)
                self.posesTable.setItem(row_count, 0, QTableWidgetItem(axes[i][j]))
                self.posesTable.setItem(row_count, 1, QTableWidgetItem(str(pose[i][j])))
    
    def grid_checkbox_changed(self, state):
        rospy.loginfo('%s::%s:' % (self._name, self.get_function_name()))
        if state == Qt.Checked:
            self.gridTable.setEnabled(True)
            self.set_current_grid()
        elif state == Qt.Unchecked:
            self.gridTable.setEnabled(False)
            self.remove_grid()

    def press_add_joint_pose(self):
        rospy.loginfo('%s::%s:' % (self._name, self.get_function_name()))
        
        if self._received_joint_states == False:
            rospy.logwarn('%s::%s: cannot add joint pose because I have not received it' % (self._name, self.get_function_name()))
            return
        
        self.add_joint_pose(list(self.joint_state.position)) # create new list, otherwise a reference is added,
        self.set_current_step()
        
    def press_add_pose(self):
        try:
            self._listener.waitForTransform(self.end_effector, self.base_link, rospy.Time(0.), rospy.Duration(1.))
            (trans, quat) = self._listener.lookupTransform(self.end_effector, self.base_link, rospy.Time(0.))
        except tf.Exception:
            rospy.logwarn('%s::%s: cannot add pose because I cannot read it' % (self._name, self.get_function_name()))
            return
        
        euler = tf.transformations.euler_from_quaternion([quat[0], quat[1], quat[2], quat[3]])
        self.add_pose([list(trans),list(euler)])
        self.set_current_step()

        
    def press_delete_joint_pose(self):
        rospy.loginfo('%s::%s:' % (self._name, self.get_function_name()))
        current_joint_pose_index = self.jointPosesBox.currentIndex()
        
        self.delete_joint_pose(current_joint_pose_index)
        self.set_current_step()
   
    
    def press_delete_pose(self):
        rospy.loginfo('%s::%s:' % (self._name, self.get_function_name()))
        current_pose_index = self.posesBox.currentIndex()

        self.delete_pose(current_pose_index)
        self.set_current_step()

    def check_grid_table_value(self, item):
        item_list = ['X', 'Y', 'x_first', 'x_step', 'y_step']
        try:
            string_error = ""
            
            if item.text() == "":
                item.setText(str(self.step['grid'][item_list[item.row()]]))
            
            if self.gridTable.verticalHeaderItem(item.row()).text().lower().find('size') != -1:
                string_error = "is not an integer"
                value = int(item.text())
            elif self.gridTable.verticalHeaderItem(item.row()).text().lower().find('step') != -1:
                string_error = "is not a number"
                value = float(item.text())
            elif self.gridTable.verticalHeaderItem(item.row()).text().lower().find('first') != -1:
                string_error = "is not true or false"
                if item.text().lower() == 'true':
                    value = True
                elif item.text().lower() == 'false':
                    value = False
                else:
                    raise ValueError('')
            
            self.step['grid'][item_list[item.row()]] = value
            
        except ValueError, e:
            rospy.logwarn('%s::%s: %s %s!' % (self._name, self.get_function_name(), item.text(), string_error))
            item.setText("")
        except AttributeError, e:
            pass

    def grid_item_activated(self, item):
        self.check_grid_table_value(item)

    def grid_table_changed(self, current, previous):
        self.check_grid_table_value(previous)
        self.check_grid_table_value(current)
        
    def jointstate_cb(self, js):
        
        for name in js.name:
            if name in self.joint_state.name:
                index_rec = js.name.index(name)
                if index_rec < len(js.position):
                    index = self.joint_state.name.index(name)
                    self.joint_state.position[index] = js.position[index_rec]
                    
        self._received_joint_states = all([position != None for position in self.joint_state.position])
                
    def get_function_name(self):
        return inspect.currentframe().f_back.f_code.co_name
