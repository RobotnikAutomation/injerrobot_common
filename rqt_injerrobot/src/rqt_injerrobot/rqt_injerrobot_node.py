#!/usr/bin/python

import os
import inspect

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QDialog, QFileDialog, QMessageBox, QTableWidgetItem

import yaml
from rqt_injerrobot_goto_dialog import RqtGoto

class RqtInjerrobot(Plugin):

    def __init__(self, context):
        super(RqtInjerrobot, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('RqtInjerrobot')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_injerrobot'), 'resource', 'RqtInjerrobot.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('RqtInjerrobotUi')
        
        # add signals/slots
        self._widget.selectYamlPushButton.pressed.connect(self.press_select_yaml)
        self._widget.loadYamlPushButton.pressed.connect(self.press_load_yaml)
        self._widget.saveYamlPushButton.pressed.connect(self.press_save_yaml)
        
        self._widget.armBox.currentIndexChanged.connect(self.arm_selected)
        self._widget.armBox.highlighted.connect(self.arm_activated)
        
        self._widget.editStepPushButton.pressed.connect(self.press_edit_step)
        
        self._widget.actionNSEdit.editingFinished.connect(self.edited_actionns)
        self._widget.groupNameEdit.editingFinished.connect(self.edited_groupname)
        self._widget.baseLinkEdit.editingFinished.connect(self.edited_baselink)
        self._widget.endEffectorEdit.editingFinished.connect(self.edited_endeffector)
        
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        self._yaml_file = ""
        self._params = dict()
        self._name = "RqtInjerrobot"
        
        self._keys_not_steps = ['arm_ip', 'arm_port', 'joint_names', 'group_name', 'action_ns']
        
        self._goto_dialog = RqtGoto()

# controller
    def edited_actionns(self):
        self.set_current_actionns(self._widget.actionNSEdit.text())
        
    def edited_groupname(self):
        self.set_current_movegroup_name(self._widget.groupNameEdit.text())

    def edited_baselink(self):
        self.set_current_base_link(self._widget.baseLinkEdit.text())
        
    def edited_endeffector(self):
        self.set_current_end_effector(self._widget.endEffectorEdit.text())

    def arm_activated(self, val):
        print 'arm_activated', val
        
    def get_arm_names(self):
        return self._params.keys()
        
    def get_current_arm_name(self):
        current_arm_name = self._widget.armBox.currentText()
        
        if current_arm_name in self.get_arm_names():
            return current_arm_name
        return None

    def get_steps(self):
        steps = None
        try:
            steps = [k for k in self._params[self.get_current_arm_name()].keys() if k not in self._keys_not_steps]
        except KeyError, e:
            rospy.logerr('%s::%s: : cannot get steps for %s. %s' % (self._name, self.get_function_name(), self.get_current_arm_name(), e))
        return steps
    
    def get_current_step_name(self):
        current_step = self._widget.stepBox.currentText()
        
        if current_step in self.get_steps():
            return current_step
        return None

    def get_current_step(self):
        step = None
        try:
            step = self._params[self.get_current_arm_name()][self.get_current_step_name()]
        except KeyError, e:
            rospy.logerr('%s::%s: : cannot get step for %s. %s' % (self._name, self.get_function_name(), self.get_current_arm_name(), e))
        return step

        
        
    def get_current_base_link(self):
        base_link = None
        try:
            base_link = self._params[self.get_current_arm_name()]['base_link']
        except KeyError, e:
            rospy.logerr('%s::%s: : cannot get base link name for %s. %s' % (self._name, self.get_function_name(), self.get_current_arm_name(), e))
        return base_link

    def set_current_base_link(self, base_link):
        self._params[self.get_current_arm_name()]['base_link'] = base_link
            

    def get_current_end_effector(self):
        end_effector = None
        try:
            end_effector = self._params[self.get_current_arm_name()]['end_effector']
        except KeyError, e:
            rospy.logerr('%s::%s: : cannot get end effector name for %s. %s' % (self._name, self.get_function_name(), self.get_current_arm_name(), e))
        return end_effector
    
    def set_current_end_effector(self, end_effector):
        self._params[self.get_current_arm_name()]['end_effector'] = end_effector
    
    
    def get_current_movegroup_name(self):
        movegroup_name = None
        try:
            movegroup_name = self._params[self.get_current_arm_name()]['group_name']
        except KeyError, e:
            rospy.logerr('%s::%s: : cannot get move group name for %s. %s' % (self._name, self.get_function_name(), self.get_current_arm_name(), e))
        return movegroup_name
    
    def set_current_movegroup_name(self, movegroup_name):
        self._params[self.get_current_arm_name()]['group_name'] = movegroup_name
        
        
    def get_current_actionns(self):
        actionns_name = None
        try:
            actionns_name = self._params[self.get_current_arm_name()]['action_ns']
        except KeyError, e:
            rospy.logerr('%s::%s: : cannot get action namespace name for %s. %s' % (self._name, self.get_function_name(), self.get_current_arm_name(), e))
        return actionns_name
    
    def set_current_actionns(self, actionns):
        self._params[self.get_current_arm_name()]['action_ns'] = actionns
    
    
    def get_current_joint_names(self):
        joint_names = None
        try:
            joint_names = self._params[self.get_current_arm_name()]['joint_names']
        except KeyError, e:
            rospy.logerr('%s::%s: : cannot get joint names for %s. %s' % (self._name, self.get_function_name(), self.get_current_arm_name(), e))
        return joint_names

    def get_current_inputs(self):
        inputs = None
        try:
            inputs = self._params[self.get_current_arm_name()]['input']
        except KeyError, e:
            rospy.logerr('%s::%s: : cannot get inputs for %s. %s' % (self._name, self.get_function_name(), self.get_current_arm_name(), e))
        return inputs

    def get_current_outputs(self):
        outputs = None
        try:
            outputs = self._params[self.get_current_arm_name()]['output']
        except KeyError, e:
            rospy.logerr('%s::%s: : cannot get outputs for %s. %s' % (self._name, self.get_function_name(), self.get_current_arm_name(), e))
        return outputs

    def set_current_step(self, step):
        self._params[self.get_current_arm_name()][self.get_current_step_name()] = step

    def set_current_arm(self): # this is view...
        self._widget.groupNameEdit.setText(self.get_current_movegroup_name())
        self._widget.actionNSEdit.setText(self.get_current_actionns())
        self._widget.baseLinkEdit.setText(self.get_current_base_link())
        self._widget.endEffectorEdit.setText(self.get_current_end_effector())
        
        
        joint_names = self.get_current_joint_names()
        self._widget.jointsTable.setRowCount(len(joint_names))
        for i in range(len(joint_names)):
            self._widget.jointsTable.setItem(i, 0, QTableWidgetItem(joint_names[i]))
        
        inputs = self.get_current_inputs()
        self._widget.inputsTable.setRowCount(0)
        for input_key in inputs: # different ways of filling the tabele
            row_count = self._widget.inputsTable.rowCount()
            self._widget.inputsTable.insertRow(row_count)
            self._widget.inputsTable.setItem(row_count, 0, QTableWidgetItem(input_key))
            self._widget.inputsTable.setItem(row_count, 1, QTableWidgetItem(str(inputs[input_key])))

        outputs = self.get_current_outputs()
        self._widget.outputsTable.setRowCount(0)
        for output_key in outputs: # different ways of filling the tabele
            row_count = self._widget.outputsTable.rowCount()
            self._widget.outputsTable.insertRow(row_count)
            self._widget.outputsTable.setItem(row_count, 0, QTableWidgetItem(output_key))
            self._widget.outputsTable.setItem(row_count, 1, QTableWidgetItem(str(outputs[output_key])))
        
        self._widget.stepBox.clear()
        steps = self.get_steps()
        for s in steps:
            self._widget.stepBox.addItem(s)
        self._widget.stepBox.setCurrentIndex(0)
            
# model
    def parse_params(self):
        self._widget.armBox.clear()
        arm_names = self.get_arm_names()
        for arm in arm_names:
            self._widget.armBox.addItem(arm)
        self._widget.armBox.setCurrentIndex(0)
    
    def load_yaml(self):
        try:
            with open(self._yaml_file, 'r') as stream:
                try:
                    rospy.loginfo('%s::%s: opening file %s' % (self._name, self.get_function_name(), self._yaml_file))
                    self._params = yaml.load(stream)
                    self.parse_params()
                
                except yaml.YAMLError, e:
                    rospy.logerr('%s::%s: : error parsing yaml file %s. %s' % (self._name, self.get_function_name(), self._yaml_file, e))
                    return -1
        except IOError, e:
            rospy.logerr('%s::%s: : error opening yaml file %s. %s' % (self._name, self.get_function_name(), self._yaml_file, e))
            return -1
              
              
    def save_yaml(self):
        try:
            with open(self._yaml_file, 'w') as stream:
                try:
                    rospy.loginfo('%s::%s: writing to file %s' % (self._name, self.get_function_name(), self._yaml_file))
                    yaml.safe_dump(self._params, stream)
                except yaml.YAMLError, e:
                    rospy.logerr('%s::%s: : error dumping yaml file %s. %s' % (self._name, self.get_function_name(), self._yaml_file, e))
                    return -1
        except IOError, e:
            rospy.logerr('%s::%s: : error opening yaml file %s. %s' % (self._name, self.get_function_name(), self._yaml_file, e))
            return -1
                
                        
    def set_yaml(self, new_yaml_file):
        if new_yaml_file == None or new_yaml_file == "":
            return
            
        self._yaml_file = new_yaml_file
        self._widget.yamlFileEdit.setText(self._yaml_file)
        
        rospy.loginfo('%s::%s: set file %s' % (self._name, self.get_function_name(), self._yaml_file))


# iu callbacks # view
    def press_select_yaml(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontConfirmOverwrite
        select_yaml_caption = "Select YAML configuration file"
        select_yaml_directory = ""
        select_yaml_file_filter = "YAML Files (*.yaml);;All Files (*)"
        yaml_file, file_filter = QFileDialog.getSaveFileName(self._widget, caption=select_yaml_caption, directory=select_yaml_directory, filter=select_yaml_file_filter, options=options)
        # yaml_file, file_filter = QFileDialog.getOpenFileName(self._widget, caption=select_yaml_caption, directory=select_yaml_directory, filter=select_yaml_file_filter, options=options)
        rospy.loginfo('%s::%s: selected file %s' % (self._name, self.get_function_name(), yaml_file))
        self.set_yaml(yaml_file)

    def press_load_yaml(self):
        ret = QMessageBox.question(self._widget, "Load YAML", 'Do you want to load a new YAML and discard current changes?', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            self.load_yaml()

    def press_save_yaml(self):
        ret = QMessageBox.question(self._widget, "Save YAML", 'Do you want to save changes to current YAML file, overwitting its content?', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            self.save_yaml()

    def press_edit_step(self):
        self._goto_dialog.step = self.get_current_step()
        self._goto_dialog.base_link = self.get_current_base_link()
        self._goto_dialog.end_effector = self.get_current_end_effector()
        self._goto_dialog.set_joint_names(self.get_current_joint_names())
        
        result = self._goto_dialog.exec_()
        
        if result == QDialog.Accepted:
            self.set_current_step(self._goto_dialog.step)
    
    def arm_selected(self, index):
        self.set_current_arm()

        
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
    
    def get_function_name(self):
        return inspect.currentframe().f_back.f_code.co_name
    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
