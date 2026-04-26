#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""GroundingDINO-assisted pixel-to-plane calibration.

This keeps the original Sagittarius calibration idea:
1. Move the end effector to known table-plane coordinates.
2. The operator places the block under the gripper, so the block coordinate is known.
3. Move the arm to a camera observation pose.
4. Detect the block center in the image.
5. Fit vision_config.yaml from paired robot/image points.
"""

import csv
import os
import sys
import time

import actionlib
import cv2
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from sagittarius_object_color_detector.msg import SGRCtrlAction, SGRCtrlGoal, SGRCtrlResult

from manual_vision_calibration import (
    fit_line,
    load_yaml,
    mean_abs_error,
    save_yaml_with_backup,
)
from perception_framework.backend_factory import create_backend
from perception_framework.backends.base import BackendConfig
from perception_framework.decision import evaluate_target_selection
from perception_framework.visualization import draw_detection_overlay


DEFAULT_POINTS = [
    (0.250, 0.000),
    (0.225, 0.025),
    (0.275, 0.025),
    (0.275, -0.025),
    (0.225, -0.025),
]


class LanguageGuidedCalibration:
    def __init__(self):
        package_dir = rospkg.RosPack().get_path("sagittarius_object_color_detector")
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_header = None

        self.arm_name = rospy.get_param("~arm_name", "sgr532")
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.target_text = rospy.get_param("~target_text", "red block").strip()
        self.min_score = float(rospy.get_param("~min_grasp_score", 0.35))
        self.min_target_text_length = int(rospy.get_param("~min_target_text_length", 2))
        self.max_detection_attempts = int(rospy.get_param("~max_detection_attempts", 8))
        self.detection_retry_sec = float(rospy.get_param("~detection_retry_sec", 0.5))
        self.observation_settle_sec = float(rospy.get_param("~observation_settle_sec", 1.0))
        self.update_vision_config = self._get_bool_param("~update_vision_config", True)

        self.output_csv = rospy.get_param(
            "~output_csv",
            os.path.join(package_dir, "config", "manual_calibration_points.csv"),
        )
        self.vision_config = rospy.get_param(
            "~vision_config",
            os.path.join(package_dir, "config", "vision_config.yaml"),
        )
        self.image_dir = rospy.get_param(
            "~image_dir",
            "/tmp/language_guided_calibration",
        )
        self.save_images = self._get_bool_param("~save_images", True)

        # Calibration only needs a known x/y projection. Keeping the gripper
        # higher than the original HSV script avoids many self-collision plans.
        self.place_z = float(rospy.get_param("~place_z", 0.12))
        self.place_z_fallbacks = self._parse_float_list(
            rospy.get_param("~place_z_fallbacks", "0.12,0.15,0.18")
        )
        self.place_roll = float(rospy.get_param("~place_roll", 0.0))
        self.place_pitch = float(rospy.get_param("~place_pitch", 1.57))
        self.place_yaw = float(rospy.get_param("~place_yaw", 0.0))
        self.observe_x = float(rospy.get_param("~observe_x", 0.20))
        self.observe_y = float(rospy.get_param("~observe_y", 0.00))
        self.observe_z = float(rospy.get_param("~observe_z", 0.15))
        self.observe_roll = float(rospy.get_param("~observe_roll", 0.0))
        self.observe_pitch = float(rospy.get_param("~observe_pitch", 1.57))
        self.observe_yaw = float(rospy.get_param("~observe_yaw", 0.0))
        self.move_observation_via_intermediate = self._get_bool_param(
            "~move_observation_via_intermediate",
            False,
        )
        self.allow_direct_observation_fallback = self._get_bool_param(
            "~allow_direct_observation_fallback",
            True,
        )
        self.intermediate_x = float(
            rospy.get_param("~intermediate_x", self.observe_x)
        )
        self.intermediate_y = float(
            rospy.get_param("~intermediate_y", 0.0)
        )
        self.intermediate_z = float(
            rospy.get_param("~intermediate_z", max(self.observe_z, 0.23))
        )
        self.intermediate_roll = float(
            rospy.get_param("~intermediate_roll", self.observe_roll)
        )
        self.intermediate_pitch = float(
            rospy.get_param("~intermediate_pitch", self.observe_pitch)
        )
        self.intermediate_yaw = float(
            rospy.get_param("~intermediate_yaw", 0.0)
        )

        self.points = self._parse_points(rospy.get_param("~calibration_points", ""))

        backend_config = BackendConfig(
            name=rospy.get_param("~perception_backend", "grounding_dino"),
            device=rospy.get_param("~device", "cuda"),
            box_threshold=float(rospy.get_param("~box_threshold", 0.35)),
            text_threshold=float(rospy.get_param("~text_threshold", 0.25)),
            model_config=rospy.get_param(
                "~groundingdino_config",
                rospy.get_param("~perception_config", ""),
            ),
            model_weights=rospy.get_param(
                "~groundingdino_weights",
                rospy.get_param("~perception_weights", ""),
            ),
        )
        self.backend = create_backend(backend_config)

        action_name = "{}/sgr_ctrl".format(self.arm_name)
        self.client = actionlib.SimpleActionClient(action_name, SGRCtrlAction)
        rospy.loginfo("Waiting for action server: %s", action_name)
        self.client.wait_for_server()

        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )

        os.makedirs(os.path.dirname(os.path.abspath(self.output_csv)), exist_ok=True)
        if self.save_images:
            os.makedirs(self.image_dir, exist_ok=True)

    def run(self):
        rospy.loginfo("Waiting for camera image on %s", self.image_topic)
        self._wait_for_image()

        collected = []
        print("")
        print("语言标定开始。每一步机械臂会先移动到已知桌面坐标。")
        print("请把红色方块放到夹爪正下方，然后按回车。")
        print("检测目标文本: '{}'".format(self.target_text))
        print("")

        for index, (robot_x, robot_y) in enumerate(self.points, start=1):
            if rospy.is_shutdown():
                break
            print("========== 标定点 {}/{} ==========".format(index, len(self.points)))
            print("机械臂将移动到 robot_x={:.3f}, robot_y={:.3f}".format(robot_x, robot_y))
            self._move_place_pose(
                robot_x,
                robot_y,
            )
            self._prompt("请把方块放到夹爪正下方，放好后按回车继续...")

            rospy.loginfo("Moving to observation pose for image capture")
            self._move_to_observation_pose()
            rospy.sleep(self.observation_settle_sec)

            detection = self._detect_current_point(index)
            if detection is None:
                retry = input("本点未检测到目标。输入 r 重试，s 跳过，q 退出: ").strip().lower()
                if retry == "q":
                    break
                if retry == "s":
                    continue
                detection = self._detect_current_point(index)
                if detection is None:
                    print("重试后仍未检测到，本点跳过。")
                    continue

            pixel_x, pixel_y, score, label = detection
            collected.append(
                {
                    "pixel_x": pixel_x,
                    "pixel_y": pixel_y,
                    "robot_x": robot_x,
                    "robot_y": robot_y,
                    "score": score,
                    "label": label,
                }
            )
            print(
                "记录成功: pixel=({:.1f}, {:.1f}), robot=({:.3f}, {:.3f}), score={:.3f}, label={}".format(
                    pixel_x,
                    pixel_y,
                    robot_x,
                    robot_y,
                    score,
                    label,
                )
            )
            print("")

        if len(collected) < 2:
            raise RuntimeError("有效标定点少于 2 个，无法拟合 vision_config.yaml")

        self._write_csv(collected)
        self._fit_and_optionally_update(collected)

    def _detect_current_point(self, index):
        for attempt in range(1, self.max_detection_attempts + 1):
            image = self.latest_image.copy()
            result = self.backend.infer(image, self.target_text)
            decision = evaluate_target_selection(
                result,
                self.target_text,
                self.min_score,
                self.min_target_text_length,
            )
            if self.save_images:
                raw_path = os.path.join(self.image_dir, "point_{:02d}_raw.jpg".format(index))
                annotated_path = os.path.join(
                    self.image_dir,
                    "point_{:02d}_annotated.jpg".format(index),
                )
                cv2.imwrite(raw_path, image)
                annotated = draw_detection_overlay(image, result, self.target_text, decision)
                cv2.imwrite(annotated_path, annotated)

            if decision.should_execute:
                center = decision.selected_box.center
                return (
                    float(center[0]),
                    float(center[1]),
                    float(decision.selected_box.score),
                    decision.selected_box.label,
                )

            rospy.logwarn(
                "Calibration point %d detection attempt %d/%d failed: %s (%s)",
                index,
                attempt,
                self.max_detection_attempts,
                decision.status,
                decision.reason,
            )
            rospy.sleep(self.detection_retry_sec)
        return None

    def _write_csv(self, points):
        with open(self.output_csv, "w", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=("pixel_x", "pixel_y", "robot_x", "robot_y", "score", "label"),
            )
            writer.writeheader()
            for point in points:
                writer.writerow(point)
        print("已写入标定 CSV: {}".format(self.output_csv))

    def _fit_and_optionally_update(self, points):
        pixel_x = [point["pixel_x"] for point in points]
        pixel_y = [point["pixel_y"] for point in points]
        robot_x = [point["robot_x"] for point in points]
        robot_y = [point["robot_y"] for point in points]

        k1, b1 = fit_line(pixel_y, robot_x)
        k2, b2 = fit_line(pixel_x, robot_y)
        x_error = mean_abs_error(pixel_y, robot_x, k1, b1)
        y_error = mean_abs_error(pixel_x, robot_y, k2, b2)

        print("")
        print("拟合结果:")
        print("robot_x = k1 * pixel_y + b1")
        print("  k1 = {:.10f}".format(k1))
        print("  b1 = {:.10f}".format(b1))
        print("  mean abs error x = {:.6f} m".format(x_error))
        print("robot_y = k2 * pixel_x + b2")
        print("  k2 = {:.10f}".format(k2))
        print("  b2 = {:.10f}".format(b2))
        print("  mean abs error y = {:.6f} m".format(y_error))

        if not self.update_vision_config:
            print("update_vision_config=false，未写入 vision_config.yaml")
            return

        content = load_yaml(self.vision_config)
        content["LinearRegression"]["k1"] = float(k1)
        content["LinearRegression"]["b1"] = float(b1)
        content["LinearRegression"]["k2"] = float(k2)
        content["LinearRegression"]["b2"] = float(b2)
        backup_path = save_yaml_with_backup(self.vision_config, content)
        print("已更新: {}".format(self.vision_config))
        print("已备份: {}".format(backup_path))

    def _move_xyz_rpy(self, x, y, z, roll, pitch, yaw):
        goal = SGRCtrlGoal()
        goal.grasp_type = goal.GRASP_NONE
        goal.action_type = goal.ACTION_TYPE_XYZ_RPY
        goal.pos_x = float(x)
        goal.pos_y = float(y)
        goal.pos_z = float(z)
        goal.pos_roll = float(roll)
        goal.pos_pitch = float(pitch)
        goal.pos_yaw = float(yaw)
        self.client.send_goal_and_wait(goal, rospy.Duration.from_sec(30.0))
        result = self.client.get_result()
        if result is None or result.result != SGRCtrlResult.SUCCESS:
            raise RuntimeError("机械臂移动失败，result={}".format(None if result is None else result.result))

    def _move_place_pose(self, x, y):
        errors = []
        z_candidates = []
        for value in [self.place_z] + self.place_z_fallbacks:
            if value not in z_candidates:
                z_candidates.append(value)

        for z in z_candidates:
            try:
                rospy.loginfo(
                    "Moving to calibration placement pose x=%.3f, y=%.3f, z=%.3f",
                    x,
                    y,
                    z,
                )
                self._move_xyz_rpy(
                    x,
                    y,
                    z,
                    self.place_roll,
                    self.place_pitch,
                    self.place_yaw,
                )
                print("已到达放置参考点，高度 z={:.3f}".format(z))
                return
            except RuntimeError as exc:
                errors.append("z={:.3f}: {}".format(z, exc))
                rospy.logwarn(
                    "Placement pose failed at z=%.3f, trying a safer height if available: %s",
                    z,
                    exc,
                )
        raise RuntimeError(
            "所有放置参考高度都规划失败。请先用 demo_true/RViz 把机械臂移动到正常无遮挡姿态，"
            "或重新运行时增加 _place_z_fallbacks:=0.15,0.18,0.22。失败详情: {}".format(
                "; ".join(errors)
            )
        )

    def _move_to_observation_pose(self):
        direct_error = None
        if self.move_observation_via_intermediate:
            rospy.loginfo(
                "Moving through intermediate pose before observation xyz=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f)",
                self.intermediate_x,
                self.intermediate_y,
                self.intermediate_z,
                self.intermediate_roll,
                self.intermediate_pitch,
                self.intermediate_yaw,
            )
            try:
                self._move_xyz_rpy(
                    self.intermediate_x,
                    self.intermediate_y,
                    self.intermediate_z,
                    self.intermediate_roll,
                    self.intermediate_pitch,
                    self.intermediate_yaw,
                )
            except RuntimeError as exc:
                if not self.allow_direct_observation_fallback:
                    raise RuntimeError(
                        "机械臂无法到达中间安全观察位: {}".format(exc)
                    )
                rospy.logwarn(
                    "Intermediate pose failed, trying direct observation pose because allow_direct_observation_fallback=true: %s",
                    exc,
                )

        try:
            self._move_xyz_rpy(
                self.observe_x,
                self.observe_y,
                self.observe_z,
                self.observe_roll,
                self.observe_pitch,
                self.observe_yaw,
            )
            return
        except RuntimeError as exc:
            direct_error = exc

        raise RuntimeError(
            "机械臂无法到达观察位。可尝试调大 intermediate_z 或调整 intermediate_yaw。失败详情: {}".format(
                direct_error
            )
        )

    def _wait_for_image(self):
        rate = rospy.Rate(10)
        start_time = time.time()
        while not rospy.is_shutdown() and self.latest_image is None:
            if time.time() - start_time > 15.0:
                raise RuntimeError("15 秒内没有收到相机图像: {}".format(self.image_topic))
            rate.sleep()

    def _image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_header = msg.header
        except CvBridgeError as exc:
            rospy.logwarn("CvBridge conversion failed during calibration: %s", exc)

    def _parse_points(self, points_text):
        if not points_text:
            return list(DEFAULT_POINTS)
        points = []
        for item in points_text.split(";"):
            item = item.strip()
            if not item:
                continue
            x_text, y_text = item.split(",", 1)
            points.append((float(x_text), float(y_text)))
        if len(points) < 2:
            raise ValueError("calibration_points must contain at least two points")
        return points

    def _parse_float_list(self, values_text):
        values = []
        for item in str(values_text).split(","):
            item = item.strip()
            if item:
                values.append(float(item))
        return values

    def _prompt(self, message):
        try:
            input(message)
        except EOFError:
            print(message)
            rospy.sleep(3.0)

    def _get_bool_param(self, name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)


def main():
    rospy.init_node("language_guided_calibration_node", anonymous=False)
    node = LanguageGuidedCalibration()
    node.run()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
