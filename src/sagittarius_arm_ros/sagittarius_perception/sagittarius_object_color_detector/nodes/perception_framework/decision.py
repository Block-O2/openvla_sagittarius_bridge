#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dataclasses import dataclass
from typing import Optional

from perception_framework.detection_types import DetectionBox, DetectionResult
from perception_framework.selection import select_highest_score


STATUS_SELECTED = "selected"
STATUS_NO_DETECTION = "no_detection"
STATUS_LOW_CONFIDENCE = "low_confidence"
STATUS_INVALID_TARGET = "invalid_target"


@dataclass
class SelectionDecision:
    """Backend-agnostic decision about whether a detection may trigger grasping."""

    status: str
    reason: str
    selected_box: Optional[DetectionBox] = None
    candidate_count: int = 0
    min_score: float = 0.0

    @property
    def should_execute(self) -> bool:
        return self.status == STATUS_SELECTED and self.selected_box is not None


def evaluate_target_selection(
    result: Optional[DetectionResult],
    target_text: str,
    min_score: float,
    min_target_text_length: int = 2,
) -> SelectionDecision:
    """Choose the best candidate and classify safe non-execution cases.

    GroundingDINO already filters candidates with its own box/text thresholds. This
    extra decision layer makes the downstream grasp trigger explicit and safe.
    """

    normalized_target = (target_text or "").strip()
    if len(normalized_target) < min_target_text_length:
        return SelectionDecision(
            status=STATUS_INVALID_TARGET,
            reason="target text is empty or too short",
            min_score=min_score,
        )

    if result is None or not result.boxes:
        return SelectionDecision(
            status=STATUS_NO_DETECTION,
            reason="no detection matched the requested target",
            candidate_count=0,
            min_score=min_score,
        )

    selected = select_highest_score(result)
    if selected is None:
        return SelectionDecision(
            status=STATUS_NO_DETECTION,
            reason="no selectable detection candidate",
            candidate_count=len(result.boxes),
            min_score=min_score,
        )

    if selected.score < min_score:
        return SelectionDecision(
            status=STATUS_LOW_CONFIDENCE,
            reason="best candidate score {:.3f} is below min_grasp_score {:.3f}".format(
                selected.score,
                min_score,
            ),
            selected_box=selected,
            candidate_count=len(result.boxes),
            min_score=min_score,
        )

    return SelectionDecision(
        status=STATUS_SELECTED,
        reason="selected highest-confidence candidate",
        selected_box=selected,
        candidate_count=len(result.boxes),
        min_score=min_score,
    )
