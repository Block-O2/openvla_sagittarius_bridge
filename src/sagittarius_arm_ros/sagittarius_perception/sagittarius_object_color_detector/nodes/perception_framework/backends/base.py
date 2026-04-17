#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict

from perception_framework.detection_types import DetectionResult


@dataclass
class BackendConfig:
    """Common backend configuration with a metadata escape hatch."""

    name: str
    device: str = "cuda"
    box_threshold: float = 0.35
    text_threshold: float = 0.25
    model_config: str = ""
    model_weights: str = ""
    extra: Dict[str, Any] = field(default_factory=dict)


class BasePerceptionBackend(ABC):
    """Minimal adapter interface for text-conditioned object detectors."""

    source_model = "base"

    def __init__(self, config: BackendConfig):
        self.config = config

    @abstractmethod
    def infer(self, image_bgr, text_prompt: str) -> DetectionResult:
        """Run perception on a BGR OpenCV image and return unified results."""

