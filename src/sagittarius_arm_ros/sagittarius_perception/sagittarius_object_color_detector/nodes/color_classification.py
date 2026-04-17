#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

from perception_framework.backends.base import BackendConfig
from perception_framework.backend_factory import create_backend
from perception_framework.coordinate_mapping import VisionPlaneMapper
from perception_framework.execution import SagittariusGraspExecutor
from perception_framework.selection import select_highest_score
from perception_framework.stability import CenterStabilityFilter


class LanguageGuidedGraspNode:
    """Orchestrates image/text input, perception, mapping, and grasp execution.

    The file and ROS node keep the legacy color_classification names for launch
    compatibility, but the implementation is now model-backend driven.
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
        selected_box = select_highest_score(result) if result else None
        if selected_box is None:
            with self.state_lock:
                if not self.busy and target_text == self.current_target_text:
                    self.stability_filter.reset()
            return

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


# Backward-compatible class alias for old notes/scripts that mention it.
GroundingDINOGraspNode = LanguageGuidedGraspNode


def main():
    rospy.init_node("color_classification_node", anonymous=False)
    try:
        LanguageGuidedGraspNode()
    except Exception as exc:
        rospy.logfatal("Failed to start color_classification_node: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
