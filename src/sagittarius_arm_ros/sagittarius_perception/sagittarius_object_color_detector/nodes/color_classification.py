#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import threading
from collections import deque

import actionlib
import cv2
import rospy
import yaml
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

from sagittarius_object_color_detector.msg import (
    SGRCtrlAction,
    SGRCtrlGoal,
    SGRCtrlResult,
)


class GroundingDINOGraspNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.state_lock = threading.Lock()

        self.arm_name = rospy.get_param("~arm_name", "sgr532")
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.target_topic = rospy.get_param("~target_topic", "/grasp_target_text")
        self.default_target_text = rospy.get_param("~default_target_text", "")
        self.box_threshold = float(rospy.get_param("~box_threshold", 0.35))
        self.text_threshold = float(rospy.get_param("~text_threshold", 0.25))
        self.device = rospy.get_param("~device", "cuda")
        self.stable_required = max(1, int(rospy.get_param("~stable_required", 5)))
        self.center_tolerance = float(rospy.get_param("~center_tolerance", 8.0))
        self.pick_z = float(rospy.get_param("~pick_z", 0.01))
        self.allow_start_without_groundingdino = self._get_bool_param(
            "~allow_start_without_groundingdino", False
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
        self.groundingdino_config = rospy.get_param("~groundingdino_config", "")
        self.groundingdino_weights = rospy.get_param("~groundingdino_weights", "")

        self.current_target_text = self._normalize_target_text(
            self.default_target_text
        )
        self.center_history = deque(maxlen=self.stable_required)
        self.busy = False
        self.last_detection_time = rospy.Time(0)
        self.groundingdino_ready = False

        self.linearression_kb_dst = self._load_vision_config(
            rospy.get_param("~vision_config")
        )
        self._load_groundingdino_model()

        action_name = "{}/sgr_ctrl".format(self.arm_name)
        self.client = actionlib.SimpleActionClient(action_name, SGRCtrlAction)
        rospy.loginfo("Waiting for action server: %s", action_name)
        self.client.wait_for_server()

        self._move_to_search_pose()

        self.target_sub = rospy.Subscriber(
            self.target_topic, String, self._target_callback, queue_size=1
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
                "GroundingDINO grasp node started with default target: '%s'",
                self.current_target_text,
            )
        else:
            rospy.loginfo(
                "GroundingDINO grasp node started, waiting for target text on %s",
                self.target_topic,
            )
        if not self.groundingdino_ready:
            rospy.logwarn(
                "GroundingDINO is not ready. Detection is disabled, but arm/camera/topic integration can still be tested."
            )

    def _load_vision_config(self, filename):
        try:
            with open(filename, "r") as f:
                content = yaml.safe_load(f)
        except Exception as exc:
            rospy.logerr("Failed to open vision config %s: %s", filename, exc)
            raise

        linearression_kb_dst = {
            "k1": float(content["LinearRegression"]["k1"]),
            "b1": float(content["LinearRegression"]["b1"]),
            "k2": float(content["LinearRegression"]["k2"]),
            "b2": float(content["LinearRegression"]["b2"]),
        }
        rospy.loginfo(
            "x axis k and b: %.6f, %.6f",
            linearression_kb_dst["k1"],
            linearression_kb_dst["b1"],
        )
        rospy.loginfo(
            "y axis k and b: %.6f, %.6f",
            linearression_kb_dst["k2"],
            linearression_kb_dst["b2"],
        )
        return linearression_kb_dst

    def _load_groundingdino_model(self):
        if not self.groundingdino_config or not self.groundingdino_weights:
            self._handle_groundingdino_unavailable(
                "groundingdino_config and groundingdino_weights must be set"
            )
            return
        if not os.path.isfile(self.groundingdino_config):
            self._handle_groundingdino_unavailable(
                "GroundingDINO config file not found: {}".format(
                    self.groundingdino_config
                )
            )
            return
        if not os.path.isfile(self.groundingdino_weights):
            self._handle_groundingdino_unavailable(
                "GroundingDINO weights file not found: {}".format(
                    self.groundingdino_weights
                )
            )
            return

        try:
            import torch
            from PIL import Image as PILImage
            import groundingdino.datasets.transforms as T
            from groundingdino.models import build_model
            from groundingdino.util.misc import clean_state_dict
            from groundingdino.util.slconfig import SLConfig
            from groundingdino.util.utils import get_phrases_from_posmap
        except ImportError as exc:
            self._handle_groundingdino_unavailable(
                "Failed to import GroundingDINO runtime. "
                "Please install the official GroundingDINO Python package. "
                "Original error: {}".format(exc)
            )
            return

        resolved_device = self.device
        if resolved_device.startswith("cuda") and not torch.cuda.is_available():
            rospy.logwarn("CUDA is unavailable, fallback to CPU for GroundingDINO")
            resolved_device = "cpu"

        self.torch = torch
        self.pil_image_cls = PILImage
        self.groundingdino_build_model = build_model
        self.groundingdino_clean_state_dict = clean_state_dict
        self.groundingdino_slconfig = SLConfig
        self.groundingdino_get_phrases_from_posmap = get_phrases_from_posmap
        self.device = resolved_device
        self.groundingdino_transform = T.Compose(
            [
                T.RandomResize([800], max_size=1333),
                T.ToTensor(),
                T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ]
        )

        rospy.loginfo("Loading GroundingDINO model on %s", self.device)
        self.groundingdino_model = self._gdino_load_model()
        self.groundingdino_ready = True

    def _gdino_preprocess_caption(self, caption):
        result = caption.lower().strip()
        return result if result.endswith(".") else result + "."

    def _gdino_load_model(self):
        args = self.groundingdino_slconfig.fromfile(self.groundingdino_config)
        args.device = self.device
        model = self.groundingdino_build_model(args)
        checkpoint = self.torch.load(self.groundingdino_weights, map_location="cpu")
        model.load_state_dict(
            self.groundingdino_clean_state_dict(checkpoint["model"]),
            strict=False,
        )
        model.eval()
        return model

    def _gdino_predict(self, image_tensor, caption):
        caption = self._gdino_preprocess_caption(caption)
        model = self.groundingdino_model.to(self.device)
        image_tensor = image_tensor.to(self.device)

        with self.torch.no_grad():
            outputs = model(image_tensor[None], captions=[caption])

        prediction_logits = outputs["pred_logits"].cpu().sigmoid()[0]
        prediction_boxes = outputs["pred_boxes"].cpu()[0]

        mask = prediction_logits.max(dim=1)[0] > self.box_threshold
        logits = prediction_logits[mask]
        boxes = prediction_boxes[mask]

        tokenizer = model.tokenizer
        tokenized = tokenizer(caption)
        phrases = [
            self.groundingdino_get_phrases_from_posmap(
                logit > self.text_threshold,
                tokenized,
                tokenizer,
            ).replace(".", "")
            for logit in logits
        ]
        return boxes, logits.max(dim=1)[0], phrases

    def _normalize_target_text(self, text):
        return text.strip()

    def _get_bool_param(self, name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _handle_groundingdino_unavailable(self, reason):
        if self.allow_start_without_groundingdino:
            rospy.logwarn("%s", reason)
            rospy.logwarn(
                "allow_start_without_groundingdino=true, node will continue without detection capability"
            )
            self.groundingdino_ready = False
            return
        raise RuntimeError(reason)

    def _target_callback(self, msg):
        new_target = self._normalize_target_text(msg.data)
        with self.state_lock:
            self.current_target_text = new_target
            self.center_history.clear()

        if new_target:
            rospy.loginfo("Updated grasp target text: '%s'", new_target)
        else:
            rospy.loginfo("Cleared grasp target text")

    def _image_callback(self, msg):
        now = rospy.Time.now()
        with self.state_lock:
            if self.busy:
                return
            if not self.groundingdino_ready:
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

        detection = self._detect_target(cv_image, target_text)
        if detection is None:
            with self.state_lock:
                if not self.busy and target_text == self.current_target_text:
                    self.center_history.clear()
            return

        start_grasp = False
        stable_center = None
        with self.state_lock:
            if self.busy or target_text != self.current_target_text:
                return

            self.center_history.append(detection["center"])
            if self._is_detection_stable():
                stable_center = self._average_center()
                self.center_history.clear()
                self.busy = True
                start_grasp = True

        if start_grasp:
            rospy.loginfo(
                "Stable target '%s' detected at pixel center (%.1f, %.1f), score=%.3f",
                target_text,
                stable_center[0],
                stable_center[1],
                detection["score"],
            )
            worker = threading.Thread(
                target=self._grasp_target,
                args=(stable_center, target_text),
                daemon=True,
            )
            worker.start()

    def _detect_target(self, cv_image, target_text):
        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image_pil = self.pil_image_cls.fromarray(image_rgb)
        image_tensor, _ = self.groundingdino_transform(image_pil, None)

        try:
            boxes, logits, phrases = self._gdino_predict(image_tensor, target_text)
        except Exception as exc:
            rospy.logerr_throttle(5.0, "GroundingDINO inference failed: %s", exc)
            return None

        if boxes is None or len(boxes) == 0:
            return None

        best_index = int(self.torch.argmax(logits).item())
        best_box = boxes[best_index]
        best_score = float(logits[best_index].item())
        best_phrase = phrases[best_index] if phrases else target_text

        image_height, image_width = cv_image.shape[:2]
        bbox = self._cxcywh_to_xyxy(best_box, image_width, image_height)
        center = (
            (bbox[0] + bbox[2]) / 2.0,
            (bbox[1] + bbox[3]) / 2.0,
        )
        return {
            "bbox": bbox,
            "center": center,
            "score": best_score,
            "phrase": best_phrase,
        }

    def _cxcywh_to_xyxy(self, box, image_width, image_height):
        cx, cy, w, h = [float(v) for v in box.tolist()]
        x1 = max(0.0, (cx - w / 2.0) * image_width)
        y1 = max(0.0, (cy - h / 2.0) * image_height)
        x2 = min(float(image_width), (cx + w / 2.0) * image_width)
        y2 = min(float(image_height), (cy + h / 2.0) * image_height)
        return (x1, y1, x2, y2)

    def _is_detection_stable(self):
        if len(self.center_history) < self.stable_required:
            return False

        centers = list(self.center_history)
        for previous, current in zip(centers[:-1], centers[1:]):
            if math.hypot(current[0] - previous[0], current[1] - previous[1]) > self.center_tolerance:
                return False
        return True

    def _average_center(self):
        centers = list(self.center_history)
        avg_x = sum(center[0] for center in centers) / len(centers)
        avg_y = sum(center[1] for center in centers) / len(centers)
        return avg_x, avg_y

    def _pixel_to_arm_plane(self, center):
        grasp_x = self.linearression_kb_dst["k1"] * center[1] + self.linearression_kb_dst["b1"]
        grasp_y = self.linearression_kb_dst["k2"] * center[0] + self.linearression_kb_dst["b2"]
        return grasp_x, grasp_y

    def _send_goal(self, goal, timeout_sec=30.0):
        self.client.send_goal_and_wait(goal, rospy.Duration.from_sec(timeout_sec))
        result = self.client.get_result()
        if result is None:
            rospy.logerr("sgr_ctrl returned no result")
            return None
        return result.result

    def _move_to_search_pose(self):
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

    def _execute_pick(self, grasp_x, grasp_y):
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

    def _execute_drop(self):
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

    def _grasp_target(self, center, target_text):
        grasp_success = False

        try:
            grasp_x, grasp_y = self._pixel_to_arm_plane(center)
            rospy.loginfo(
                "Target '%s' mapped to arm plane x=%.4f, y=%.4f, z=%.4f",
                target_text,
                grasp_x,
                grasp_y,
                self.pick_z,
            )

            grasp_success = self._execute_pick(grasp_x, grasp_y)
            if grasp_success:
                rospy.loginfo("Grasp succeeded for target '%s'", target_text)
                if self.drop_after_grasp:
                    drop_success = self._execute_drop()
                    if drop_success:
                        rospy.loginfo("Drop succeeded at fixed position")
                    else:
                        rospy.logwarn(
                            "Drop failed after grasp success, target will still be cleared"
                        )
            else:
                rospy.logwarn("Grasp failed for target '%s'", target_text)
        finally:
            self._move_to_search_pose()
            with self.state_lock:
                if grasp_success and self.current_target_text == target_text:
                    self.current_target_text = ""
                self.center_history.clear()
                self.busy = False


def main():
    rospy.init_node("color_classification_node", anonymous=False)
    try:
        GroundingDINOGraspNode()
    except Exception as exc:
        rospy.logfatal("Failed to start color_classification_node: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
