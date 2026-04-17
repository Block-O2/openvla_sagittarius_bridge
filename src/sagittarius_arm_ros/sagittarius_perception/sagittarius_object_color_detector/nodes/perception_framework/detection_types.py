#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


Point2D = Tuple[float, float]
BBoxXYXY = Tuple[float, float, float, float]
ImageSize = Tuple[int, int]


@dataclass
class DetectionBox:
    """One backend-agnostic detection candidate."""

    bbox_xyxy: BBoxXYXY
    score: float
    label: str = ""
    metadata: Dict[str, Any] = field(default_factory=dict)

    @property
    def center(self) -> Point2D:
        x1, y1, x2, y2 = self.bbox_xyxy
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0


@dataclass
class DetectionResult:
    """Unified output consumed by selection, mapping, and execution logic."""

    source_model: str
    timestamp: float
    image_size: ImageSize
    boxes: List[DetectionBox] = field(default_factory=list)
    selected_box: Optional[DetectionBox] = None
    mask: Any = None
    metadata: Dict[str, Any] = field(default_factory=dict)

    @property
    def selected_center(self) -> Optional[Point2D]:
        if self.selected_box is None:
            return None
        return self.selected_box.center

    @property
    def labels(self) -> List[str]:
        return [box.label for box in self.boxes]

    @property
    def scores(self) -> List[float]:
        return [box.score for box in self.boxes]

