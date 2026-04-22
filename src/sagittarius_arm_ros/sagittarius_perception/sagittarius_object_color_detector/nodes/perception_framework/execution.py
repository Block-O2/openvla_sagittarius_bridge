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

    def move_to_search_pose(self, mode="none"):
        """Optionally move to an explicit search pose.

        The default is intentionally no motion. Earlier versions treated
        "stay" as the legacy DEFINE_STAY preset, but that preset is a fixed
        joint posture and can surprise the operator. Keeping the current pose
        is safer for live demos unless the user explicitly requests a preset.
        """
        mode = (mode or "none").strip().lower()
        if mode in ("none", "disabled", "off", "stay", "hold", "hold_current"):
            rospy.loginfo("Keeping current robot pose because search_pose_mode=%s", mode)
            return True

        goal = SGRCtrlGoal()
        goal.grasp_type = goal.GRASP_NONE
        if mode in ("define_stay", "preset_stay"):
            goal.action_type = goal.ACTION_TYPE_DEFINE_STAY
        elif mode in ("xyz_rpy", "search", "legacy"):
            goal.action_type = goal.ACTION_TYPE_XYZ_RPY
            goal.pos_x = 0.2
            goal.pos_y = 0.0
            goal.pos_z = 0.15
            goal.pos_pitch = 1.57
            goal.pos_yaw = 0.0
        else:
            rospy.logwarn(
                "Unknown search_pose_mode=%s, keeping current pose instead of using a hard-coded preset",
                mode,
            )
            return True

        result = self._send_goal(goal)
        if result != SGRCtrlResult.SUCCESS:
            rospy.logwarn("Failed to move to search pose mode '%s', result=%s", mode, result)
            return False
        rospy.loginfo("Moved to search/startup pose mode '%s'", mode)
        return True

    def execute_pick(self, grasp_x, grasp_y, orientation_mode="auto"):
        goal = SGRCtrlGoal()
        goal.grasp_type = goal.GRASP_OPEN
        goal.pos_x = grasp_x
        goal.pos_y = grasp_y
        goal.pos_z = self.pick_z

        orientation_mode = (orientation_mode or "auto").strip().lower()
        if orientation_mode in ("fixed", "fixed_rpy", "xyz_rpy", "legacy"):
            goal.action_type = goal.ACTION_TYPE_PICK_XYZ_RPY
            goal.pos_pitch = 1.57
            rospy.loginfo("Executing pick with fixed legacy RPY orientation")
        else:
            goal.action_type = goal.ACTION_TYPE_PICK_XYZ
            rospy.loginfo("Executing pick with dynamic orientation from target XYZ")

        result = self._send_goal(goal)
        if result == SGRCtrlResult.SUCCESS:
            return True
        if result == SGRCtrlResult.PLAN_NOT_FOUND and goal.action_type == goal.ACTION_TYPE_PICK_XYZ:
            rospy.logwarn("Pick XYZ planning failed, retry with fixed legacy PICK_XYZ_RPY")
            goal.action_type = goal.ACTION_TYPE_PICK_XYZ_RPY
            goal.pos_pitch = 1.57
            result = self._send_goal(goal)
            return result == SGRCtrlResult.SUCCESS
        if result == SGRCtrlResult.PLAN_NOT_FOUND and goal.action_type == goal.ACTION_TYPE_PICK_XYZ_RPY:
            rospy.logwarn("Pick XYZ_RPY planning failed, retry with dynamic PICK_XYZ")
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
