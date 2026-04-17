#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from perception_framework.detection_types import DetectionResult


def select_highest_score(result: DetectionResult):
    """Select the highest-confidence candidate in-place and return it."""
    if not result.boxes:
        result.selected_box = None
        return None
    result.selected_box = max(result.boxes, key=lambda box: box.score)
    return result.selected_box

