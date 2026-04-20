#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import rospy
from std_msgs.msg import String

from perception_framework.coordinate_mapping import VisionPlaneMapper
from perception_framework.execution import SagittariusGraspExecutor
from perception_framework.observation import (
    build_execution_feedback,
    dumps_feedback,
    loads_observation,
)
from perception_framework.stability import CenterStabilityFilter


STATE_WAITING_FOR_OBSERVATION = "waiting_for_observation"
STATE_TARGET_LOCKED = "target_locked"
STATE_GRASPING = "grasping"
STATE_PLACING = "placing"
STATE_DONE = "done"
STATE_FAILED = "failed"


class LanguageGuidedExecutorNode:
    """Robot station node: target observation JSON -> mapping -> sgr_ctrl."""

    def __init__(self):
        self.state_lock = threading.Lock()

        self.arm_name = rospy.get_param("~arm_name", "sgr532")
        self.observation_topic = rospy.get_param(
            "~observation_topic",
            "/language_guided_grasp/target_observation",
        )
        self.execution_feedback_topic = rospy.get_param(
            "~execution_feedback_topic",
            "/language_guided_grasp/execution_feedback",
        )
        self.state_topic = rospy.get_param(
            "~state_topic",
            "/language_guided_executor/state",
        )
        self.min_grasp_score = float(rospy.get_param("~min_grasp_score", 0.35))
        self.stable_required = max(1, int(rospy.get_param("~stable_required", 5)))
        self.center_tolerance = float(rospy.get_param("~center_tolerance", 8.0))
        self.max_observation_age = float(rospy.get_param("~max_observation_age", 0.0))
        self.pick_z = float(rospy.get_param("~pick_z", 0.01))
        self.drop_after_grasp = self._get_bool_param("~drop_after_grasp", True)
        self.drop_position = (
            float(rospy.get_param("~drop_x", 0.15)),
            float(rospy.get_param("~drop_y", 0.24)),
            float(rospy.get_param("~drop_z", 0.20)),
        )
        self.ignore_completed_target_until_changed = self._get_bool_param(
            "~ignore_completed_target_until_changed",
            True,
        )

        self.busy = False
        self.completed_target_text = ""
        self.pipeline_state = None
        self.pipeline_state_reason = ""
        self.last_selected_payload = None

        self.mapper = VisionPlaneMapper(rospy.get_param("~vision_config"))
        rospy.loginfo(
            "Robot station mapper x k/b: %.6f, %.6f",
            self.mapper.params["k1"],
            self.mapper.params["b1"],
        )
        rospy.loginfo(
            "Robot station mapper y k/b: %.6f, %.6f",
            self.mapper.params["k2"],
            self.mapper.params["b2"],
        )
        self.stability_filter = CenterStabilityFilter(
            self.stable_required,
            self.center_tolerance,
        )
        self.executor = SagittariusGraspExecutor(
            arm_name=self.arm_name,
            pick_z=self.pick_z,
            drop_position=self.drop_position,
        )
        self.executor.move_to_search_pose()

        self.feedback_pub = rospy.Publisher(
            self.execution_feedback_topic,
            String,
            queue_size=1,
        )
        self.state_pub = rospy.Publisher(
            self.state_topic,
            String,
            queue_size=1,
            latch=True,
        )
        self.observation_sub = rospy.Subscriber(
            self.observation_topic,
            String,
            self._observation_callback,
            queue_size=1,
        )
        self._set_state(STATE_WAITING_FOR_OBSERVATION, "startup")
        rospy.loginfo(
            "Robot station executor ready: observation=%s feedback=%s",
            self.observation_topic,
            self.execution_feedback_topic,
        )

    def _observation_callback(self, msg):
        try:
            observation = loads_observation(msg.data)
        except ValueError as exc:
            rospy.logwarn("Invalid target observation JSON: %s", exc)
            return

        now = rospy.Time.now().to_sec()
        source_stamp = observation.get("source_stamp")
        if (
            self.max_observation_age > 0.0
            and source_stamp is not None
            and now - float(source_stamp) > self.max_observation_age
        ):
            self._fail_safely("stale_observation")
            return

        status = observation.get("status", "")
        target_text = self._normalize_target_text(observation.get("target_text", ""))
        selected = observation.get("selected")

        with self.state_lock:
            if self.busy:
                rospy.loginfo_throttle(
                    2.0,
                    "Robot station is busy; ignoring incoming observation",
                )
                return
            if (
                self.ignore_completed_target_until_changed
                and self.completed_target_text
                and target_text == self.completed_target_text
            ):
                return
            if self.completed_target_text and target_text != self.completed_target_text:
                self.completed_target_text = ""

        if status != "selected" or not selected:
            self._fail_safely(status or "not_selected")
            return

        score = float(selected.get("score", 0.0))
        if score < self.min_grasp_score:
            self._fail_safely("low_confidence")
            return

        center = selected.get("center")
        if not self._valid_center(center):
            self._fail_safely("invalid_center")
            return

        center = (float(center[0]), float(center[1]))
        start_grasp = False
        stable_center = None
        with self.state_lock:
            if self.busy:
                return
            self.stability_filter.add(center)
            self.last_selected_payload = selected
            if self.stability_filter.is_stable():
                stable_center = self.stability_filter.average_center()
                self.stability_filter.reset()
                self.busy = True
                self._set_state(STATE_TARGET_LOCKED, "stable remote observation")
                start_grasp = True

        if start_grasp:
            rospy.loginfo(
                "Robot station locked target '%s' at pixel center (%.1f, %.1f), score=%.3f",
                target_text,
                stable_center[0],
                stable_center[1],
                score,
            )
            worker = threading.Thread(
                target=self._execute_target,
                args=(stable_center, target_text, selected),
                daemon=True,
            )
            worker.start()

    def _execute_target(self, center, target_text, selected):
        grasp_success = False
        self._set_state(STATE_GRASPING, "executing pick")
        try:
            grasp_x, grasp_y = self.mapper.map_pixel_center(center)
            rospy.loginfo(
                "Robot station mapped '%s' to arm plane x=%.4f, y=%.4f, z=%.4f",
                target_text,
                grasp_x,
                grasp_y,
                self.pick_z,
            )
            grasp_success = self.executor.execute_pick(grasp_x, grasp_y)
            if grasp_success and self.drop_after_grasp:
                self._set_state(STATE_PLACING, "executing fixed drop")
                if not self.executor.execute_drop():
                    rospy.logwarn("Drop failed after successful pick")
        finally:
            self.executor.move_to_search_pose()
            status = "done" if grasp_success else "failed"
            reason = "grasp succeeded" if grasp_success else "grasp failed"
            self.feedback_pub.publish(
                String(
                    data=dumps_feedback(
                        build_execution_feedback(
                            status,
                            target_text=target_text,
                            reason=reason,
                            selected=selected,
                        )
                    )
                )
            )
            with self.state_lock:
                if grasp_success and self.ignore_completed_target_until_changed:
                    self.completed_target_text = target_text
                self.stability_filter.reset()
                self.busy = False
                if grasp_success:
                    self._set_state(STATE_DONE, reason)
                else:
                    self._set_state(STATE_FAILED, reason)

    def _fail_safely(self, reason):
        with self.state_lock:
            if not self.busy:
                self.stability_filter.reset()
                self._set_state(STATE_FAILED, reason)
        rospy.loginfo_throttle(3.0, "Robot station safe no-grasp: %s", reason)

    def _valid_center(self, center):
        return (
            isinstance(center, list)
            and len(center) == 2
            and center[0] is not None
            and center[1] is not None
        )

    def _set_state(self, state, reason=""):
        if (
            self.pipeline_state == state
            and self.pipeline_state_reason == reason
        ):
            return
        self.pipeline_state = state
        self.pipeline_state_reason = reason
        state_text = "{}: {}".format(state, reason) if reason else state
        rospy.loginfo("Robot station state -> %s", state_text)
        if hasattr(self, "state_pub"):
            self.state_pub.publish(String(data=state_text))

    def _normalize_target_text(self, text):
        return (text or "").strip()

    def _get_bool_param(self, name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)


def main():
    rospy.init_node("language_guided_executor_node", anonymous=False)
    try:
        LanguageGuidedExecutorNode()
    except Exception as exc:
        rospy.logfatal("Failed to start language_guided_executor_node: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
