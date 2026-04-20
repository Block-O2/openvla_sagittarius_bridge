#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time


OBSERVATION_SCHEMA_VERSION = 1
FEEDBACK_SCHEMA_VERSION = 1


def build_target_observation(result, decision, target_text, source_stamp=None):
    """Convert a perception decision into a lightweight JSON-ready dict."""

    observation = {
        "schema_version": OBSERVATION_SCHEMA_VERSION,
        "message_type": "target_observation",
        "created_time": time.time(),
        "source_stamp": source_stamp,
        "source_model": getattr(result, "source_model", "") if result else "",
        "target_text": target_text,
        "status": decision.status,
        "reason": decision.reason,
        "candidate_count": decision.candidate_count,
        "min_score": decision.min_score,
        "image_width": 0,
        "image_height": 0,
        "boxes": [],
        "selected": None,
    }

    if result is not None:
        observation["image_width"] = int(result.image_size[0])
        observation["image_height"] = int(result.image_size[1])
        observation["boxes"] = [
            {
                "bbox_xyxy": [float(value) for value in box.bbox_xyxy],
                "center": [float(box.center[0]), float(box.center[1])],
                "score": float(box.score),
                "label": box.label,
            }
            for box in result.boxes
        ]

    if decision.selected_box is not None:
        selected = decision.selected_box
        observation["selected"] = {
            "bbox_xyxy": [float(value) for value in selected.bbox_xyxy],
            "center": [float(selected.center[0]), float(selected.center[1])],
            "score": float(selected.score),
            "label": selected.label,
        }

    return observation


def dumps_observation(observation):
    return json.dumps(observation, sort_keys=True)


def loads_observation(payload):
    return json.loads(payload)


def build_execution_feedback(status, target_text="", reason="", selected=None):
    return {
        "schema_version": FEEDBACK_SCHEMA_VERSION,
        "message_type": "execution_feedback",
        "created_time": time.time(),
        "status": status,
        "target_text": target_text,
        "reason": reason,
        "selected": selected,
    }


def dumps_feedback(feedback):
    return json.dumps(feedback, sort_keys=True)


def loads_feedback(payload):
    return json.loads(payload)
