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
from perception_framework.coordinate_mapping import VisionPlaneMapper
from perception_framework.decision import evaluate_target_selection
from perception_framework.execution import SagittariusGraspExecutor
from perception_framework.stability import CenterStabilityFilter
from perception_framework.visualization import draw_detection_overlay


class LanguageGuidedGraspNode:
    """Orchestrates image/text input, perception, mapping, and grasp execution.

    This is the canonical node for the text-target to grasp pipeline. The old
    color_classification.py entry point remains as a compatibility wrapper.
    """

    def __init__(self):
        self.bridge = CvBridge()
        self.state_lock = threading.Lock()

        self.arm_name = rospy.get_param("~arm_name", "sgr532")
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.target_topic = rospy.get_param("~target_topic", "/grasp_target_text")
        self.default_target_text = rospy.get_param("~default_target_text", "")
        self.perception_backend_name = rospy.get_param(
            "~perception_backend", "grounding_dino"
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
        self.stable_required = max(1, int(rospy.get_param("~stable_required", 5)))
        self.center_tolerance = float(rospy.get_param("~center_tolerance", 8.0))
        self.pick_z = float(rospy.get_param("~pick_z", 0.01))
        self.allow_start_without_backend = self._get_bool_param(
            "~allow_start_without_groundingdino", False
        )
        self.allow_start_without_backend = self._get_bool_param(
            "~allow_start_without_backend", self.allow_start_without_backend
        )
        self.drop_after_grasp = self._get_bool_param("~drop_after_grasp", True)
        self.drop_position = (
            float(rospy.get_param("~drop_x", 0.15)),
            float(rospy.get_param("~drop_y", 0.24)),
            float(rospy.get_param("~drop_z", 0.20)),
        )
        self.min_detection_interval = float(
            rospy.get_param("~min_detection_interval", 0.2)
        )
        self.publish_annotated_image = self._get_bool_param(
            "~publish_annotated_image", True
        )
        self.annotated_image_topic = rospy.get_param(
            "~annotated_image_topic", "/language_guided_grasp/annotated_image"
        )
        self.save_annotated_image = self._get_bool_param(
            "~save_annotated_image", False
        )
        self.annotated_image_path = rospy.get_param(
            "~annotated_image_path", "/tmp/language_guided_grasp_latest.jpg"
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
        self.busy = False
        self.backend_ready = False
        self.last_detection_time = rospy.Time(0)

        self.mapper = VisionPlaneMapper(rospy.get_param("~vision_config"))
        rospy.loginfo(
            "x axis k and b: %.6f, %.6f",
            self.mapper.params["k1"],
            self.mapper.params["b1"],
        )
        rospy.loginfo(
            "y axis k and b: %.6f, %.6f",
            self.mapper.params["k2"],
            self.mapper.params["b2"],
        )

        self.stability_filter = CenterStabilityFilter(
            self.stable_required,
            self.center_tolerance,
        )
        self.backend = self._create_perception_backend()

        self.executor = SagittariusGraspExecutor(
            arm_name=self.arm_name,
            pick_z=self.pick_z,
            drop_position=self.drop_position,
        )
        self.executor.move_to_search_pose()

        self.target_sub = rospy.Subscriber(
            self.target_topic,
            String,
            self._target_callback,
            queue_size=1,
        )
        self.annotated_image_pub = None
        if self.publish_annotated_image:
            self.annotated_image_pub = rospy.Publisher(
                self.annotated_image_topic,
                Image,
                queue_size=1,
            )
        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )

        if self.current_target_text:
            rospy.loginfo(
                "Language-guided grasp node started with default target: '%s'",
                self.current_target_text,
            )
        else:
            rospy.loginfo(
                "Language-guided grasp node started, waiting for target text on %s",
                self.target_topic,
            )
        rospy.loginfo(
            "Safe selection enabled: min_grasp_score=%.3f, annotated_image_topic=%s",
            self.min_grasp_score,
            self.annotated_image_topic if self.publish_annotated_image else "disabled",
        )
        if not self.backend_ready:
            rospy.logwarn(
                "Perception backend is not ready. Detection is disabled, but arm/camera/topic integration can still be tested."
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
                rospy.logwarn(
                    "allow_start_without_backend=true, node will continue without detection capability"
                )
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
            self.stability_filter.reset()

        if new_target:
            rospy.loginfo("Updated grasp target text: '%s'", new_target)
        else:
            rospy.loginfo("Cleared grasp target text")

    def _image_callback(self, msg):
        now = rospy.Time.now()
        with self.state_lock:
            if self.busy or not self.backend_ready:
                return
            target_text = self.current_target_text
            if not target_text:
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
        self._publish_detection_observation(cv_image, result, target_text, decision, msg)

        if not decision.should_execute:
            self._log_selection_decision(target_text, decision)
            with self.state_lock:
                if not self.busy and target_text == self.current_target_text:
                    self.stability_filter.reset()
            return

        selected_box = decision.selected_box
        start_grasp = False
        stable_center = None
        with self.state_lock:
            if self.busy or target_text != self.current_target_text:
                return

            self.stability_filter.add(selected_box.center)
            if self.stability_filter.is_stable():
                stable_center = self.stability_filter.average_center()
                self.stability_filter.reset()
                self.busy = True
                start_grasp = True

        if start_grasp:
            rospy.loginfo(
                "Stable target '%s' detected by %s at pixel center (%.1f, %.1f), score=%.3f, label='%s'",
                target_text,
                result.source_model,
                stable_center[0],
                stable_center[1],
                selected_box.score,
                selected_box.label,
            )
            worker = threading.Thread(
                target=self._grasp_target,
                args=(stable_center, target_text),
                daemon=True,
            )
            worker.start()

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

    def _log_selection_decision(self, target_text, decision):
        rospy.loginfo_throttle(
            3.0,
            "Target '%s' not selected for grasp: status=%s, candidates=%d, reason=%s",
            target_text,
            decision.status,
            decision.candidate_count,
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

    def _grasp_target(self, center, target_text):
        grasp_success = False
        try:
            grasp_x, grasp_y = self.mapper.map_pixel_center(center)
            rospy.loginfo(
                "Target '%s' mapped to arm plane x=%.4f, y=%.4f, z=%.4f",
                target_text,
                grasp_x,
                grasp_y,
                self.pick_z,
            )

            grasp_success = self.executor.execute_pick(grasp_x, grasp_y)
            if grasp_success:
                rospy.loginfo("Grasp succeeded for target '%s'", target_text)
                if self.drop_after_grasp:
                    if self.executor.execute_drop():
                        rospy.loginfo("Drop succeeded at fixed position")
                    else:
                        rospy.logwarn(
                            "Drop failed after grasp success, target will still be cleared"
                        )
            else:
                rospy.logwarn("Grasp failed for target '%s'", target_text)
        finally:
            self.executor.move_to_search_pose()
            with self.state_lock:
                if grasp_success and self.current_target_text == target_text:
                    self.current_target_text = ""
                self.stability_filter.reset()
                self.busy = False

    def _normalize_target_text(self, text):
        return text.strip()

    def _get_bool_param(self, name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)


# Backward-compatible class alias for older notes/scripts that mention it.
GroundingDINOGraspNode = LanguageGuidedGraspNode


def main():
    rospy.init_node("language_guided_grasp_node", anonymous=False)
    try:
        LanguageGuidedGraspNode()
    except Exception as exc:
        rospy.logfatal("Failed to start language_guided_grasp_node: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
