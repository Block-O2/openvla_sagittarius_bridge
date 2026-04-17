#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import actionlib
import rospy

from sagittarius_object_color_detector.msg import (
    SGRCtrlAction,
    SGRCtrlGoal,
    SGRCtrlResult,
)


class SagittariusGraspExecutor:
    """Thin wrapper around the existing sgr_ctrl action pipeline."""

    def __init__(self, arm_name: str, pick_z: float, drop_position):
        self.pick_z = float(pick_z)
        self.drop_position = drop_position
        action_name = "{}/sgr_ctrl".format(arm_name)
        self.client = actionlib.SimpleActionClient(action_name, SGRCtrlAction)
        rospy.loginfo("Waiting for action server: %s", action_name)
        self.client.wait_for_server()

    def move_to_search_pose(self):
        goal = SGRCtrlGoal()
        goal.grasp_type = goal.GRASP_NONE
        goal.action_type = goal.ACTION_TYPE_XYZ_RPY
        goal.pos_x = 0.2
        goal.pos_y = 0.0
        goal.pos_z = 0.15
        goal.pos_pitch = 1.57
        goal.pos_yaw = 0.0
        result = self._send_goal(goal)
        if result != SGRCtrlResult.SUCCESS:
            rospy.logwarn("Failed to move to search pose, result=%s", result)
            return False
        return True

    def execute_pick(self, grasp_x, grasp_y):
        goal = SGRCtrlGoal()
        goal.grasp_type = goal.GRASP_OPEN
        goal.action_type = goal.ACTION_TYPE_PICK_XYZ_RPY
        goal.pos_x = grasp_x
        goal.pos_y = grasp_y
        goal.pos_z = self.pick_z
        goal.pos_pitch = 1.57

        result = self._send_goal(goal)
        if result == SGRCtrlResult.SUCCESS:
            return True
        if result == SGRCtrlResult.PLAN_NOT_FOUND:
            rospy.logwarn("Pick XYZ_RPY planning failed, retry with PICK_XYZ")
            goal.action_type = goal.ACTION_TYPE_PICK_XYZ
            result = self._send_goal(goal)
            return result == SGRCtrlResult.SUCCESS

        if result == SGRCtrlResult.GRASP_FAILD:
            rospy.logwarn("Pick failed because the gripper did not hold the object")
        else:
            rospy.logwarn("Pick action returned result=%s", result)
        return False

    def execute_drop(self):
        goal = SGRCtrlGoal()
        goal.action_type = goal.ACTION_TYPE_PUT_XYZ
        goal.pos_x = self.drop_position[0]
        goal.pos_y = self.drop_position[1]
        goal.pos_z = self.drop_position[2]
        result = self._send_goal(goal)
        if result != SGRCtrlResult.SUCCESS:
            rospy.logwarn("Drop action failed, result=%s", result)
            return False
        return True

    def _send_goal(self, goal, timeout_sec=30.0):
        self.client.send_goal_and_wait(goal, rospy.Duration.from_sec(timeout_sec))
        result = self.client.get_result()
        if result is None:
            rospy.logerr("sgr_ctrl returned no result")
            return None
        return result.result
