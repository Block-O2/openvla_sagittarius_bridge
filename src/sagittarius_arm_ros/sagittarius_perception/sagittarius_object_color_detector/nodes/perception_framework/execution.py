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

    def __init__(self, arm_name: str, pick_z: float, drop_position, search_pose=None):
        self.pick_z = float(pick_z)
        self.drop_position = drop_position
        self.search_pose = search_pose or {
            "x": 0.20,
            "y": 0.00,
            "z": 0.15,
            "roll": 0.0,
            "pitch": 1.57,
            "yaw": 0.0,
        }
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

        if mode in ("define_stay", "preset_stay"):
            rospy.logwarn(
                "search_pose_mode=%s uses legacy joint preset and may point the camera away from the table; prefer camera_down",
                mode,
            )
            goal = SGRCtrlGoal()
            goal.grasp_type = goal.GRASP_NONE
            goal.action_type = goal.ACTION_TYPE_DEFINE_STAY
        elif mode in ("camera_down", "table_view", "down", "xyz_rpy", "search", "legacy"):
            return self.move_to_pose(self.search_pose, "camera/table search pose")
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

    def move_to_pose(self, pose, label="observation pose"):
        goal = SGRCtrlGoal()
        goal.grasp_type = goal.GRASP_NONE
        goal.action_type = goal.ACTION_TYPE_XYZ_RPY
        goal.pos_x = float(pose["x"])
        goal.pos_y = float(pose["y"])
        goal.pos_z = float(pose["z"])
        goal.pos_roll = float(pose["roll"])
        goal.pos_pitch = float(pose["pitch"])
        goal.pos_yaw = float(pose["yaw"])
        rospy.loginfo(
            "Moving to %s xyz=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f)",
            label,
            goal.pos_x,
            goal.pos_y,
            goal.pos_z,
            goal.pos_roll,
            goal.pos_pitch,
            goal.pos_yaw,
        )
        result = self._send_goal(goal)
        if result != SGRCtrlResult.SUCCESS:
            rospy.logwarn("Failed to move to %s, result=%s", label, result)
            return False
        rospy.loginfo("Moved to %s", label)
        return True

    def move_to_pose_via_intermediate(
        self,
        pose,
        intermediate_pose,
        label="observation pose",
        allow_direct_fallback=True,
    ):
        if intermediate_pose:
            moved = self.move_to_pose(
                intermediate_pose,
                "{} intermediate pose".format(label),
            )
            if not moved:
                if not allow_direct_fallback:
                    return False
                rospy.logwarn(
                    "Failed to move to %s intermediate pose; trying direct %s because allow_direct_fallback=true",
                    label,
                    label,
                )
        return self.move_to_pose(pose, label)

    def move_to_pose_with_retries(
        self,
        pose,
        label="observation pose",
        retries=1,
        retry_interval=0.2,
        intermediate_pose=None,
        allow_direct_fallback=True,
    ):
        attempts = max(1, int(retries))
        for attempt in range(1, attempts + 1):
            if intermediate_pose is not None:
                moved = self.move_to_pose_via_intermediate(
                    pose,
                    intermediate_pose,
                    label=label,
                    allow_direct_fallback=allow_direct_fallback,
                )
            else:
                moved = self.move_to_pose(pose, label)
            if moved:
                return True
            if attempt < attempts:
                rospy.logwarn(
                    "Retrying move to %s (%d/%d) after %.2fs",
                    label,
                    attempt + 1,
                    attempts,
                    retry_interval,
                )
                if retry_interval > 0.0:
                    rospy.sleep(retry_interval)
        return False

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
        return self.execute_drop_at(
            self.drop_position[0],
            self.drop_position[1],
            self.drop_position[2],
        )

    def execute_drop_at(self, drop_x, drop_y, drop_z):
        goal = SGRCtrlGoal()
        goal.action_type = goal.ACTION_TYPE_PUT_XYZ
        goal.pos_x = float(drop_x)
        goal.pos_y = float(drop_y)
        goal.pos_z = float(drop_z)
        result = self._send_goal(goal)
        if result != SGRCtrlResult.SUCCESS:
            rospy.logwarn(
                "Drop action failed at x=%.4f, y=%.4f, z=%.4f, result=%s",
                goal.pos_x,
                goal.pos_y,
                goal.pos_z,
                result,
            )
            return False
        return True

    def execute_rejection_gesture(self, yaw_delta=0.30, cycles=1, pause_sec=0.15):
        base_pose = dict(self.search_pose)
        poses = []
        for _ in range(max(1, int(cycles))):
            left_pose = dict(base_pose)
            left_pose["yaw"] = float(base_pose["yaw"]) + float(yaw_delta)
            right_pose = dict(base_pose)
            right_pose["yaw"] = float(base_pose["yaw"]) - float(yaw_delta)
            poses.extend([left_pose, right_pose])
        poses.append(base_pose)

        success = True
        for index, pose in enumerate(poses, start=1):
            moved = self.move_to_pose(pose, "rejection gesture step {}".format(index))
            success = success and moved
            if pause_sec > 0.0:
                rospy.sleep(pause_sec)
        return success

    def _send_goal(self, goal, timeout_sec=30.0):
        self.client.send_goal_and_wait(goal, rospy.Duration.from_sec(timeout_sec))
        result = self.client.get_result()
        if result is None:
            rospy.logerr("sgr_ctrl returned no result")
            return None
        return result.result
