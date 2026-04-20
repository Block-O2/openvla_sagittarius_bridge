#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

from perception_framework.backends.base import BackendConfig
from perception_framework.backend_factory import create_backend
from perception_framework.decision import evaluate_target_selection
from perception_framework.observation import (
    build_target_observation,
    dumps_observation,
    loads_feedback,
)
from perception_framework.visualization import draw_detection_overlay


STATE_WAITING_FOR_TARGET = "waiting_for_target"
STATE_DETECTING = "detecting"
STATE_SELECTED = "selected"
STATE_DONE = "done"
STATE_FAILED = "failed"


class LanguageGuidedPerceptionNode:
    """AI station node: image/text -> GroundingDINO -> target observation JSON."""

    def __init__(self):
        self.bridge = CvBridge()
        self.state_lock = threading.Lock()

        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.target_topic = rospy.get_param("~target_topic", "/grasp_target_text")
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
            "/language_guided_perception/state",
        )
        self.default_target_text = rospy.get_param("~default_target_text", "")
        self.perception_backend_name = rospy.get_param(
            "~perception_backend",
            "grounding_dino",
        )
        self.device = rospy.get_param("~device", "cuda")
        self.box_threshold = float(rospy.get_param("~box_threshold", 0.35))
        self.text_threshold = float(rospy.get_param("~text_threshold", 0.25))
        self.min_grasp_score = float(
            rospy.get_param("~min_grasp_score", self.box_threshold)
        )
        self.min_target_text_length = max(
            1,
            int(rospy.get_param("~min_target_text_length", 2)),
        )
        self.min_detection_interval = float(
            rospy.get_param("~min_detection_interval", 0.2)
        )
        self.publish_annotated_image = self._get_bool_param(
            "~publish_annotated_image",
            True,
        )
        self.annotated_image_topic = rospy.get_param(
            "~annotated_image_topic",
            "/language_guided_perception/annotated_image",
        )
        self.save_annotated_image = self._get_bool_param(
            "~save_annotated_image",
            False,
        )
        self.annotated_image_path = rospy.get_param(
            "~annotated_image_path",
            "/tmp/language_guided_perception_latest.jpg",
        )
        self.allow_start_without_backend = self._get_bool_param(
            "~allow_start_without_backend",
            False,
        )
        self.model_config = rospy.get_param(
            "~groundingdino_config",
            rospy.get_param("~perception_config", ""),
        )
        self.model_weights = rospy.get_param(
            "~groundingdino_weights",
            rospy.get_param("~perception_weights", ""),
        )

        self.current_target_text = self._normalize_target_text(
            self.default_target_text
        )
        self.backend_ready = False
        self.last_detection_time = rospy.Time(0)
        self.pipeline_state = None
        self.pipeline_state_reason = ""

        self.backend = self._create_perception_backend()

        self.observation_pub = rospy.Publisher(
            self.observation_topic,
            String,
            queue_size=1,
        )
        self.state_pub = rospy.Publisher(
            self.state_topic,
            String,
            queue_size=1,
            latch=True,
        )
        self.annotated_image_pub = None
        if self.publish_annotated_image:
            self.annotated_image_pub = rospy.Publisher(
                self.annotated_image_topic,
                Image,
                queue_size=1,
            )

        self.target_sub = rospy.Subscriber(
            self.target_topic,
            String,
            self._target_callback,
            queue_size=1,
        )
        self.feedback_sub = rospy.Subscriber(
            self.execution_feedback_topic,
            String,
            self._feedback_callback,
            queue_size=1,
        )
        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )

        initial_state = (
            STATE_DETECTING if self.current_target_text else STATE_WAITING_FOR_TARGET
        )
        self._set_state(initial_state, "startup")
        rospy.loginfo(
            "AI station perception node ready: image=%s target=%s observation=%s",
            self.image_topic,
            self.target_topic,
            self.observation_topic,
        )

    def _create_perception_backend(self):
        backend_config = BackendConfig(
            name=self.perception_backend_name,
            device=self.device,
            box_threshold=self.box_threshold,
            text_threshold=self.text_threshold,
            model_config=self.model_config,
            model_weights=self.model_weights,
        )
        try:
            backend = create_backend(backend_config)
        except Exception as exc:
            if self.allow_start_without_backend:
                rospy.logwarn("Failed to initialize perception backend: %s", exc)
                self.backend_ready = False
                return None
            raise

        self.backend_ready = True
        rospy.loginfo(
            "Loaded perception backend '%s' on %s",
            backend.source_model,
            getattr(backend, "device", self.device),
        )
        return backend

    def _target_callback(self, msg):
        new_target = self._normalize_target_text(msg.data)
        with self.state_lock:
            self.current_target_text = new_target
            if new_target:
                self._set_state(STATE_DETECTING, "new target received")
            else:
                self._set_state(STATE_WAITING_FOR_TARGET, "target cleared")
        rospy.loginfo("AI station target text: '%s'", new_target)

    def _feedback_callback(self, msg):
        try:
            feedback = loads_feedback(msg.data)
        except ValueError as exc:
            rospy.logwarn("Invalid execution feedback JSON: %s", exc)
            return

        status = feedback.get("status", "")
        target_text = self._normalize_target_text(feedback.get("target_text", ""))
        if status != "done":
            return

        with self.state_lock:
            if target_text and target_text != self.current_target_text:
                return
            self.current_target_text = ""
            self._set_state(STATE_DONE, "robot station completed target")

    def _image_callback(self, msg):
        now = rospy.Time.now()
        with self.state_lock:
            target_text = self.current_target_text
            if not target_text:
                if self.pipeline_state != STATE_WAITING_FOR_TARGET:
                    self._set_state(STATE_WAITING_FOR_TARGET, "no target text")
                return
            if not self.backend_ready:
                self._set_state(STATE_FAILED, "perception_backend_not_ready")
                return
            if self.min_detection_interval > 0.0:
                elapsed = (now - self.last_detection_time).to_sec()
                if elapsed < self.min_detection_interval:
                    return
                self.last_detection_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            rospy.logerr("CvBridge conversion failed: %s", exc)
            return

        result = self._run_perception(cv_image, target_text)
        decision = evaluate_target_selection(
            result,
            target_text,
            self.min_grasp_score,
            self.min_target_text_length,
        )
        source_stamp = msg.header.stamp.to_sec() if msg.header.stamp else None
        observation = build_target_observation(
            result,
            decision,
            target_text,
            source_stamp=source_stamp,
        )
        self.observation_pub.publish(String(data=dumps_observation(observation)))
        self._publish_detection_observation(cv_image, result, target_text, decision, msg)

        if decision.should_execute:
            self._set_state(STATE_SELECTED, "published selected target")
        else:
            self._set_state(STATE_FAILED, decision.status)
            rospy.loginfo_throttle(
                3.0,
                "AI station did not select '%s': status=%s reason=%s",
                target_text,
                decision.status,
                decision.reason,
            )

    def _run_perception(self, cv_image, target_text):
        try:
            return self.backend.infer(cv_image, target_text)
        except Exception as exc:
            rospy.logerr_throttle(
                5.0,
                "Perception backend '%s' inference failed: %s",
                self.perception_backend_name,
                exc,
            )
            return None

    def _publish_detection_observation(self, cv_image, result, target_text, decision, source_msg):
        if not self.publish_annotated_image and not self.save_annotated_image:
            return

        annotated = draw_detection_overlay(cv_image, result, target_text, decision)
        if self.annotated_image_pub is not None:
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                annotated_msg.header = source_msg.header
                self.annotated_image_pub.publish(annotated_msg)
            except CvBridgeError as exc:
                rospy.logwarn_throttle(
                    5.0,
                    "Failed to publish annotated detection image: %s",
                    exc,
                )

        if self.save_annotated_image:
            if not cv2.imwrite(self.annotated_image_path, annotated):
                rospy.logwarn_throttle(
                    5.0,
                    "Failed to save annotated detection image to %s",
                    self.annotated_image_path,
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
        rospy.loginfo("AI station state -> %s", state_text)
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
    rospy.init_node("language_guided_perception_node", anonymous=False)
    try:
        LanguageGuidedPerceptionNode()
    except Exception as exc:
        rospy.logfatal("Failed to start language_guided_perception_node: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
