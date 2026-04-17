#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from collections import deque


class CenterStabilityFilter:
    """Tracks recent centers and reports when a target is spatially stable."""

    def __init__(self, required_frames: int, tolerance_px: float):
        self.required_frames = max(1, int(required_frames))
        self.tolerance_px = float(tolerance_px)
        self._history = deque(maxlen=self.required_frames)

    def reset(self):
        self._history.clear()

    def add(self, center):
        self._history.append(center)

    def is_stable(self):
        if len(self._history) < self.required_frames:
            return False
        centers = list(self._history)
        for previous, current in zip(centers[:-1], centers[1:]):
            distance = math.hypot(
                current[0] - previous[0],
                current[1] - previous[1],
            )
            if distance > self.tolerance_px:
                return False
        return True

    def average_center(self):
        centers = list(self._history)
        avg_x = sum(center[0] for center in centers) / len(centers)
        avg_y = sum(center[1] for center in centers) / len(centers)
        return avg_x, avg_y

