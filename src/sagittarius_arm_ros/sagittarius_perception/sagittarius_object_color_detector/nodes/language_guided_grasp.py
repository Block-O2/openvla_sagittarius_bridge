#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
from dataclasses import dataclass

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
from perception_framework.task_parsing import TaskCommand, TaskStep, parse_task_command
from perception_framework.visualization import draw_detection_overlay, draw_status_banner


STATE_WAITING_FOR_TARGET = "waiting_for_target"
STATE_DETECTING = "detecting"
STATE_TARGET_LOCKED = "target_locked"
STATE_GRASPING = "grasping"
STATE_PLACING = "placing"
STATE_DONE = "done"
STATE_FAILED = "failed"


@dataclass
class TargetObservation:
    target_text: str
    view_name: str
    center: tuple
    score: float
    label: str
    arm_x: float
    arm_y: float


class LanguageGuidedGraspNode:
    """Orchestrates image/text input, perception, mapping, and grasp execution."""

    def __init__(self):
        self.bridge = CvBridge()
        self.state_lock = threading.Lock()
        self.image_lock = threading.Lock()

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
        self.execute_grasp = self._get_bool_param("~execute_grasp", True)
        self.search_pose_mode = rospy.get_param("~search_pose_mode", "none")
        self.move_to_search_pose_on_startup = self._get_bool_param(
            "~move_to_search_pose_on_startup", False
        )
        self.search_pose = {
            "x": float(rospy.get_param("~search_pose_x", 0.20)),
            "y": float(rospy.get_param("~search_pose_y", 0.00)),
            "z": float(rospy.get_param("~search_pose_z", 0.15)),
            "roll": float(rospy.get_param("~search_pose_roll", 0.0)),
            "pitch": float(rospy.get_param("~search_pose_pitch", 1.57)),
            "yaw": float(rospy.get_param("~search_pose_yaw", 0.0)),
        }
        self.return_to_search_pose_after_grasp = self._get_bool_param(
            "~return_to_search_pose_after_grasp", False
        )
        self.pick_orientation_mode = rospy.get_param("~pick_orientation_mode", "auto")
        self.drop_position = (
            float(rospy.get_param("~drop_x", 0.15)),
            float(rospy.get_param("~drop_y", 0.24)),
            float(rospy.get_param("~drop_z", 0.20)),
        )
        self.dynamic_place_z = float(
            rospy.get_param("~dynamic_place_z", self.drop_position[2])
        )
        self.rejection_motion_enabled = self._get_bool_param(
            "~rejection_motion_enabled", True
        )
        self.rejection_yaw_delta = float(
            rospy.get_param("~rejection_yaw_delta", 0.30)
        )
        self.rejection_cycles = max(1, int(rospy.get_param("~rejection_cycles", 1)))
        self.rejection_pause_sec = max(
            0.0,
            float(rospy.get_param("~rejection_pause_sec", 0.15)),
        )
        self.clear_target_after_success = self._get_bool_param(
            "~clear_target_after_success", True
        )
        self.clear_target_after_failure = self._get_bool_param(
            "~clear_target_after_failure", True
        )
        self.publish_annotated_image = self._get_bool_param(
            "~publish_annotated_image", True
        )
        self.annotated_image_topic = rospy.get_param(
            "~annotated_image_topic", "/language_guided_grasp/annotated_image"
        )
        self.state_topic = rospy.get_param(
            "~state_topic", "/language_guided_grasp/state"
        )
        self.save_annotated_image = self._get_bool_param(
            "~save_annotated_image", False
        )
        self.annotated_image_path = rospy.get_param(
            "~annotated_image_path", "/tmp/language_guided_grasp_latest.jpg"
        )
        self.save_raw_image = self._get_bool_param("~save_raw_image", False)
        self.raw_image_path = rospy.get_param(
            "~raw_image_path", "/tmp/language_guided_grasp_raw.jpg"
        )
        self.scan_attempts_per_view = max(
            1,
            int(rospy.get_param("~scan_attempts_per_view", self.stable_required)),
        )
        self.scan_stable_required = max(
            1,
            int(rospy.get_param("~scan_stable_required", self.stable_required)),
        )
        self.scan_attempts_per_view = max(
            self.scan_attempts_per_view,
            self.scan_stable_required,
        )
        self.scan_retry_interval = max(
            0.0,
            float(rospy.get_param("~scan_retry_interval", 0.2)),
        )
        self.scan_settle_sec = max(
            0.0,
            float(rospy.get_param("~scan_settle_sec", 0.8)),
        )
        self.place_front_view_enabled = self._get_bool_param(
            "~place_front_view_enabled", True
        )
        self.scan_view_order = rospy.get_param(
            "~place_scan_view_order",
            rospy.get_param("~scan_view_order", "front,left,right"),
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
        self.pending_target_text = None
        self.busy = False
        self.backend_ready = False
        self.pipeline_state = None
        self.pipeline_state_reason = ""
        self.state_pub = None
        self.latest_image = None
        self.latest_header = None
        self.pick_view = None
        self.observation_views = []

        self.executor = None
        if self.execute_grasp:
            self.executor = SagittariusGraspExecutor(
                arm_name=self.arm_name,
                pick_z=self.pick_z,
                drop_position=self.drop_position,
                search_pose=self.search_pose,
            )
            if self.move_to_search_pose_on_startup:
                self.executor.move_to_search_pose(self.search_pose_mode)
            else:
                rospy.loginfo(
                    "Startup will keep the current/home pose; observation view scanning begins after a target command arrives"
                )
        else:
            rospy.logwarn(
                "execute_grasp=false: robot action client is disabled and observation scanning will only use the current camera pose"
            )
        self.backend = self._create_perception_backend()
        mapper_cache = {}
        self.pick_view = self._build_pick_view(mapper_cache)
        self.observation_views = self._build_place_views(mapper_cache)
        if not self.execute_grasp and len(self.observation_views) > 1:
            rospy.logwarn(
                "Multiple placement views are configured, but only the current camera pose can be used when execute_grasp=false"
            )

        self.target_sub = rospy.Subscriber(
            self.target_topic,
            String,
            self._target_callback,
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
            "Pick view enabled: %s",
            self.pick_view["name"],
        )
        rospy.loginfo(
            "Placement observation views enabled: %s",
            ", ".join(view["name"] for view in self.observation_views),
        )
        rospy.loginfo(
            "Safe selection enabled: min_grasp_score=%.3f, execute_grasp=%s, annotated_image_topic=%s",
            self.min_grasp_score,
            self.execute_grasp,
            self.annotated_image_topic if self.publish_annotated_image else "disabled",
        )
        if self.save_raw_image:
            rospy.loginfo(
                "Raw camera snapshots will be saved to: %s",
                self.raw_image_path,
            )
        if self.save_annotated_image:
            rospy.loginfo(
                "Annotated detection snapshots will be saved to: %s",
                self.annotated_image_path,
            )
        rospy.loginfo("Pipeline state topic: %s", self.state_topic)
        if not self.backend_ready:
            rospy.logwarn(
                "Perception backend is not ready. Detection is disabled, but arm/camera/topic integration can still be tested."
            )

        if self.current_target_text:
            self._queue_target_processing(self.current_target_text)

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

    def _build_pick_view(self, mapper_cache):
        pick_vision_config = rospy.get_param("~vision_config")
        return self._build_view_entry(
            mapper_cache,
            "pick_front",
            pick_vision_config,
            self.search_pose,
            search_mode=self.search_pose_mode,
        )

    def _build_place_views(self, mapper_cache):
        views = {}
        if self.place_front_view_enabled:
            front_vision_config = rospy.get_param(
                "~place_front_view_vision_config",
                rospy.get_param("~vision_config"),
            )
            views["front"] = self._build_view_entry(
                mapper_cache,
                "front",
                front_vision_config,
                self._build_pose_from_params(
                    "place_front_view",
                    self.search_pose,
                ),
                search_mode="xyz_rpy",
            )

        side_defaults = {
            "left": {
                "x": 0.23,
                "y": 0.10,
                "z": 0.23,
                "roll": 0.0,
                "pitch": 1.57,
                "yaw": 1.10,
            },
            "right": {
                "x": 0.23,
                "y": -0.10,
                "z": 0.23,
                "roll": 0.0,
                "pitch": 1.57,
                "yaw": -1.10,
            },
        }
        for side in ("left", "right"):
            enabled = self._get_bool_param("~{}_view_enabled".format(side), False)
            if not enabled:
                continue
            vision_config = rospy.get_param("~{}_view_vision_config".format(side), "")
            if not vision_config:
                rospy.logwarn(
                    "%s_view_enabled=true but %s_view_vision_config is empty; disabling that view for safety",
                    side,
                    side,
                )
                continue
            pose = self._build_pose_from_params(
                "{}_view".format(side),
                side_defaults[side],
            )
            views[side] = self._build_view_entry(
                mapper_cache,
                side,
                vision_config,
                pose,
            )

        ordered_views = []
        requested_order = [
            name.strip().lower()
            for name in str(self.scan_view_order).split(",")
            if name.strip()
        ]
        for name in requested_order:
            if name in views and name not in (view["name"] for view in ordered_views):
                ordered_views.append(views[name])
        for name in ("front", "left", "right"):
            if name in views and name not in (view["name"] for view in ordered_views):
                ordered_views.append(views[name])
        return ordered_views

    def _build_pose_from_params(self, prefix, defaults):
        return {
            "x": float(rospy.get_param("~{}_x".format(prefix), defaults["x"])),
            "y": float(rospy.get_param("~{}_y".format(prefix), defaults["y"])),
            "z": float(rospy.get_param("~{}_z".format(prefix), defaults["z"])),
            "roll": float(
                rospy.get_param("~{}_roll".format(prefix), defaults["roll"])
            ),
            "pitch": float(
                rospy.get_param("~{}_pitch".format(prefix), defaults["pitch"])
            ),
            "yaw": float(rospy.get_param("~{}_yaw".format(prefix), defaults["yaw"])),
        }

    def _build_view_entry(
        self,
        mapper_cache,
        name,
        vision_config_path,
        pose,
        search_mode="xyz_rpy",
    ):
        mapper = mapper_cache.get(vision_config_path)
        if mapper is None:
            mapper = VisionPlaneMapper(vision_config_path)
            mapper_cache[vision_config_path] = mapper
            rospy.loginfo(
                "Loaded %s mapper from %s: %s",
                name,
                vision_config_path,
                mapper.describe(),
            )
            if mapper.is_degenerate():
                rospy.logwarn(
                    "%s view mapping is degenerate: pixel center changes will not change grasp x/y",
                    name,
                )
        return {
            "name": name,
            "pose": pose,
            "mapper": mapper,
            "vision_config": vision_config_path,
            "search_mode": search_mode,
        }

    def _target_callback(self, msg):
        new_target = self._normalize_target_text(msg.data)
        deferred = False
        should_start = False
        with self.state_lock:
            if self.busy:
                self.pending_target_text = new_target
                deferred = True
            else:
                self.current_target_text = new_target
                if new_target:
                    self.busy = True
                    should_start = True
                    self._set_state(STATE_DETECTING, "new task received")
                else:
                    self._set_state(STATE_WAITING_FOR_TARGET, "target cleared")

        if deferred:
            rospy.loginfo(
                "Node is busy; deferred new grasp target text: '%s'",
                new_target,
            )
            return
        if should_start:
            rospy.loginfo("Updated grasp target text: '%s'", new_target)
            self._start_worker(new_target)
        else:
            rospy.loginfo("Cleared grasp target text")

    def _start_worker(self, target_text):
        worker = threading.Thread(
            target=self._process_target_text,
            args=(target_text,),
            daemon=True,
        )
        worker.start()

    def _queue_target_processing(self, target_text):
        with self.state_lock:
            if self.busy or not target_text:
                return
            self.busy = True
            self._set_state(STATE_DETECTING, "processing default target")
        self._start_worker(target_text)

    def _image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            rospy.logerr_throttle(5.0, "CvBridge conversion failed: %s", exc)
            return
        with self.image_lock:
            self.latest_image = cv_image
            self.latest_header = msg.header

    def _process_target_text(self, target_text):
        task_success = False
        dry_run_success = False
        result_reason = "task failed"
        task = parse_task_command(target_text)

        try:
            if not self.backend_ready:
                result_reason = "perception backend not ready"
                self._set_state(STATE_FAILED, "perception_backend_not_ready")
                return

            rospy.loginfo(
                "Processing task '%s' with %d step(s): %s",
                task.raw_text,
                len(task.steps),
                self._describe_task(task),
            )

            for step_index, step in enumerate(task.steps, start=1):
                if len(step.pick_target_text) < self.min_target_text_length:
                    result_reason = "invalid pick target"
                    self._set_state(STATE_FAILED, "invalid_pick_target")
                    return
                if (
                    step.place_target_text
                    and len(step.place_target_text) < self.min_target_text_length
                ):
                    result_reason = "invalid place target"
                    self._set_state(STATE_FAILED, "invalid_place_target")
                    return

                pick_observation = self._locate_pick_target(
                    step.pick_target_text,
                    "pick step {}".format(step_index),
                )
                if pick_observation is None:
                    result_reason = "pick target '{}' not found".format(
                        step.pick_target_text
                    )
                    self._set_state(STATE_FAILED, "pick_target_not_found")
                    self._reject_current_request(result_reason)
                    return

                step_success, step_dry_run, step_reason = self._execute_task(
                    step,
                    pick_observation,
                    step_index=step_index,
                    total_steps=len(task.steps),
                )
                if not step_success and not step_dry_run:
                    result_reason = step_reason
                    return
                dry_run_success = dry_run_success or step_dry_run

            task_success = True
            result_reason = "completed {} task step(s)".format(len(task.steps))
        finally:
            self._finalize_task(
                target_text,
                task_success,
                dry_run_success,
                result_reason,
            )

    def _locate_pick_target(self, target_text, stage_name):
        if not self._move_to_pick_view():
            return None
        return self._locate_target_in_view(self.pick_view, target_text, stage_name)

    def _locate_target_across_views(self, target_text, stage_name):
        for view in self.observation_views:
            if not self._move_to_view(view):
                continue
            observation = self._locate_target_in_view(view, target_text, stage_name)
            if observation is not None:
                return observation
        return None

    def _move_to_pick_view(self):
        if self.executor is None:
            return True
        moved = self.executor.move_to_search_pose(self.pick_view["search_mode"])
        if moved and self.scan_settle_sec > 0.0:
            rospy.sleep(self.scan_settle_sec)
        return moved

    def _move_to_view(self, view):
        if self.executor is None:
            if view["name"] != "front":
                rospy.logwarn(
                    "Skipping %s view because execute_grasp=false disables robot motion",
                    view["name"],
                )
                return False
            return True

        label = "{} observation view".format(view["name"])
        moved = self.executor.move_to_pose(view["pose"], label)
        if moved and self.scan_settle_sec > 0.0:
            rospy.sleep(self.scan_settle_sec)
        return moved

    def _locate_target_in_view(self, view, target_text, stage_name):
        stability_filter = CenterStabilityFilter(
            self.scan_stable_required,
            self.center_tolerance,
        )
        for attempt in range(1, self.scan_attempts_per_view + 1):
            self._set_state(
                STATE_DETECTING,
                "scanning {} target '{}' in {} view ({}/{})".format(
                    stage_name,
                    target_text,
                    view["name"],
                    attempt,
                    self.scan_attempts_per_view,
                ),
            )
            cv_image, header = self._get_latest_image(timeout_sec=5.0)
            if cv_image is None:
                rospy.logwarn(
                    "No camera image available while scanning %s target '%s' in %s view",
                    stage_name,
                    target_text,
                    view["name"],
                )
                return None

            result = self._run_perception(cv_image, target_text)
            decision = evaluate_target_selection(
                result,
                target_text,
                self.min_grasp_score,
                self.min_target_text_length,
            )
            self._publish_detection_observation(
                cv_image,
                result,
                target_text,
                decision,
                header,
            )

            if not decision.should_execute:
                stability_filter.reset()
                self._log_selection_decision(
                    target_text,
                    decision,
                    view_name=view["name"],
                )
            else:
                selected_box = decision.selected_box
                stability_filter.add(selected_box.center)
                if stability_filter.is_stable():
                    stable_center = stability_filter.average_center()
                    arm_x, arm_y = view["mapper"].map_pixel_center(stable_center)
                    rospy.loginfo(
                        "Locked %s target '%s' in %s view at pixel center (%.1f, %.1f), mapped to x=%.4f, y=%.4f, score=%.3f, label='%s'",
                        stage_name,
                        target_text,
                        view["name"],
                        stable_center[0],
                        stable_center[1],
                        arm_x,
                        arm_y,
                        selected_box.score,
                        selected_box.label,
                    )
                    self._set_state(
                        STATE_TARGET_LOCKED,
                        "{} target locked in {} view".format(
                            stage_name,
                            view["name"],
                        ),
                    )
                    return TargetObservation(
                        target_text=target_text,
                        view_name=view["name"],
                        center=stable_center,
                        score=selected_box.score,
                        label=selected_box.label,
                        arm_x=arm_x,
                        arm_y=arm_y,
                    )

            if self.scan_retry_interval > 0.0 and attempt < self.scan_attempts_per_view:
                rospy.sleep(self.scan_retry_interval)
        return None

    def _execute_task(self, step: TaskStep, pick_observation, step_index=1, total_steps=1):
        self._set_state(
            STATE_GRASPING,
            "executing pick step {}/{} for '{}'".format(
                step_index,
                total_steps,
                step.pick_target_text,
            ),
        )
        rospy.loginfo(
            "Pick target '%s' selected from %s view: x=%.4f, y=%.4f, z=%.4f",
            step.pick_target_text,
            pick_observation.view_name,
            pick_observation.arm_x,
            pick_observation.arm_y,
            self.pick_z,
        )

        if not self.execute_grasp:
            return self._execute_dry_run_task(
                step,
                pick_observation,
                step_index=step_index,
                total_steps=total_steps,
            )

        pick_success = self.executor.execute_pick(
            pick_observation.arm_x,
            pick_observation.arm_y,
            orientation_mode=self.pick_orientation_mode,
        )
        if not pick_success:
            rospy.logwarn("Grasp failed for target '%s'", step.pick_target_text)
            return False, False, "grasp failed"

        rospy.loginfo("Grasp succeeded for target '%s'", step.pick_target_text)
        if step.place_target_text:
            place_observation = self._locate_target_across_views(
                step.place_target_text,
                "place step {}".format(step_index),
            )
            if place_observation is None:
                result_reason = "place target '{}' not found".format(
                    step.place_target_text
                )
                self._set_state(STATE_FAILED, "place_target_not_found")
                self._reject_current_request(result_reason)
                return False, False, result_reason

            rospy.loginfo(
                "Place target '%s' selected from %s view: x=%.4f, y=%.4f, z=%.4f",
                step.place_target_text,
                place_observation.view_name,
                place_observation.arm_x,
                place_observation.arm_y,
                self.dynamic_place_z,
            )
            self._set_state(
                STATE_PLACING,
                "placing step {}/{} into '{}'".format(
                    step_index,
                    total_steps,
                    step.place_target_text,
                ),
            )
            place_success = self.executor.execute_drop_at(
                place_observation.arm_x,
                place_observation.arm_y,
                self.dynamic_place_z,
            )
            if not place_success:
                rospy.logwarn(
                    "Dynamic place failed for target '%s'",
                    step.place_target_text,
                )
                return False, False, "dynamic place failed"
            rospy.loginfo(
                "Dynamic place succeeded for target '%s'",
                step.place_target_text,
            )
            return True, False, "step {}/{} pick and place succeeded".format(
                step_index,
                total_steps,
            )

        if self.drop_after_grasp:
            self._set_state(STATE_PLACING, "executing fixed drop")
            if self.executor.execute_drop():
                rospy.loginfo("Drop succeeded at fixed position")
            else:
                rospy.logwarn(
                    "Drop failed after grasp success, target will still be cleared"
                )
        return True, False, "step {}/{} grasp succeeded".format(
            step_index,
            total_steps,
        )

    def _execute_dry_run_task(self, step, pick_observation, step_index=1, total_steps=1):
        rospy.logwarn(
            "execute_grasp=false: dry run only, mapped task will not be sent to the robot"
        )
        if not step.place_target_text:
            return True, True, "dry-run pick planned"

        place_observation = self._locate_target_across_views(
            step.place_target_text,
            "place step {}".format(step_index),
        )
        if place_observation is None:
            result_reason = "place target '{}' not found".format(
                step.place_target_text
            )
            self._set_state(STATE_FAILED, "place_target_not_found")
            self._reject_current_request(result_reason)
            return False, False, result_reason

        rospy.loginfo(
            "Dry-run place target '%s' selected from %s view: x=%.4f, y=%.4f, z=%.4f",
            step.place_target_text,
            place_observation.view_name,
            place_observation.arm_x,
            place_observation.arm_y,
            self.dynamic_place_z,
        )
        return True, True, "dry-run task planned"

    def _reject_current_request(self, reason_text):
        rospy.logwarn("Rejecting current request: %s", reason_text)
        if self.execute_grasp and self.rejection_motion_enabled and self.executor:
            self.executor.execute_rejection_gesture(
                yaw_delta=self.rejection_yaw_delta,
                cycles=self.rejection_cycles,
                pause_sec=self.rejection_pause_sec,
            )
        self._publish_status_observation("rejected: {}".format(reason_text))

    def _finalize_task(self, target_text, task_success, dry_run_success, result_reason):
        next_target = None
        with self.state_lock:
            if (
                (task_success or dry_run_success)
                and self.clear_target_after_success
                and self.current_target_text == target_text
            ):
                self.current_target_text = ""
            elif (
                not task_success
                and not dry_run_success
                and self.clear_target_after_failure
                and self.current_target_text == target_text
            ):
                self.current_target_text = ""

            if self.pending_target_text is not None:
                self.current_target_text = self.pending_target_text
                next_target = self.pending_target_text
                self.pending_target_text = None

            self.busy = False

        if self.execute_grasp and self.return_to_search_pose_after_grasp and self.executor:
            self.executor.move_to_search_pose(self.search_pose_mode)

        if dry_run_success:
            self._set_state(STATE_DONE, result_reason)
        elif task_success:
            self._set_state(STATE_DONE, result_reason)
        else:
            self._set_state(STATE_FAILED, result_reason)

        if next_target:
            with self.state_lock:
                self.busy = True
            self._set_state(STATE_DETECTING, "processing deferred target")
            self._start_worker(next_target)
        elif not self.current_target_text:
            self._set_state(STATE_WAITING_FOR_TARGET, "target complete")

    def _get_latest_image(self, timeout_sec=5.0):
        deadline = time.time() + timeout_sec
        while not rospy.is_shutdown():
            with self.image_lock:
                if self.latest_image is not None:
                    return self.latest_image.copy(), self.latest_header
            if time.time() >= deadline:
                return None, None
            rospy.sleep(0.05)
        return None, None

    def _publish_detection_observation(
        self,
        cv_image,
        result,
        target_text,
        decision,
        source_header,
    ):
        if (
            not self.publish_annotated_image
            and not self.save_annotated_image
            and not self.save_raw_image
        ):
            return

        if self.save_raw_image:
            if not cv2.imwrite(self.raw_image_path, cv_image):
                rospy.logwarn_throttle(
                    5.0,
                    "Failed to save raw camera image to %s",
                    self.raw_image_path,
                )

        annotated = draw_detection_overlay(cv_image, result, target_text, decision)
        if self.annotated_image_pub is not None:
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                if source_header is not None:
                    annotated_msg.header = source_header
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

    def _publish_status_observation(self, text, banner_color=(0, 80, 220)):
        cv_image, header = self._get_latest_image(timeout_sec=0.5)
        if cv_image is None:
            return
        annotated = draw_status_banner(cv_image, text, banner_color=banner_color)
        if self.annotated_image_pub is not None:
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                if header is not None:
                    annotated_msg.header = header
                self.annotated_image_pub.publish(annotated_msg)
            except CvBridgeError as exc:
                rospy.logwarn_throttle(
                    5.0,
                    "Failed to publish status image: %s",
                    exc,
                )
        if self.save_annotated_image:
            if not cv2.imwrite(self.annotated_image_path, annotated):
                rospy.logwarn_throttle(
                    5.0,
                    "Failed to save status image to %s",
                    self.annotated_image_path,
                )

    def _log_selection_decision(self, target_text, decision, view_name="front"):
        rospy.loginfo(
            "Target '%s' not selected in %s view: status=%s, candidates=%d, reason=%s",
            target_text,
            view_name,
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

    def _set_state(self, state, reason=""):
        if self.pipeline_state == state and self.pipeline_state_reason == reason:
            return
        self.pipeline_state = state
        self.pipeline_state_reason = reason
        state_text = "{}: {}".format(state, reason) if reason else state
        rospy.loginfo("Pipeline state -> %s", state_text)
        if self.state_pub is not None:
            self.state_pub.publish(String(data=state_text))

    def _normalize_target_text(self, text):
        return text.strip()

    def _describe_task(self, task: TaskCommand):
        parts = []
        for index, step in enumerate(task.steps, start=1):
            if step.place_target_text:
                parts.append(
                    "{}: '{}' -> '{}'".format(
                        index,
                        step.pick_target_text,
                        step.place_target_text,
                    )
                )
            else:
                parts.append("{}: '{}'".format(index, step.pick_target_text))
        return "; ".join(parts)

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
