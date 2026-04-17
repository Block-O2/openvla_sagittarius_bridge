#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2

from perception_framework.decision import (
    STATUS_LOW_CONFIDENCE,
    STATUS_NO_DETECTION,
    STATUS_SELECTED,
    SelectionDecision,
)


def draw_detection_overlay(image_bgr, result, target_text, decision: SelectionDecision):
    """Return an annotated BGR image for demos and debugging."""

    annotated = image_bgr.copy()
    height, width = annotated.shape[:2]

    if decision.status == STATUS_SELECTED:
        banner_color = (40, 150, 40)
    elif decision.status == STATUS_LOW_CONFIDENCE:
        banner_color = (0, 165, 255)
    elif decision.status == STATUS_NO_DETECTION:
        banner_color = (80, 80, 220)
    else:
        banner_color = (120, 120, 120)

    _draw_banner(
        annotated,
        "target: {} | status: {} | {}".format(target_text or "<empty>", decision.status, decision.reason),
        banner_color,
    )

    if result is None:
        return annotated

    for index, box in enumerate(result.boxes):
        is_selected = box is decision.selected_box
        color = _box_color(decision.status, is_selected)
        thickness = 3 if is_selected else 1
        x1, y1, x2, y2 = _clip_box(box.bbox_xyxy, width, height)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)

        label = "{} {:.2f}".format(box.label or "candidate", box.score)
        _draw_label(annotated, label, x1, max(22, y1), color)

        if is_selected:
            cx, cy = box.center
            cx = int(round(max(0.0, min(float(width - 1), cx))))
            cy = int(round(max(0.0, min(float(height - 1), cy))))
            cv2.circle(annotated, (cx, cy), 6, color, -1)
            cv2.circle(annotated, (cx, cy), 10, (255, 255, 255), 2)
            _draw_label(annotated, "center ({}, {})".format(cx, cy), cx + 8, cy - 8, color)

        if index >= 20:
            _draw_label(annotated, "... more candidates omitted", 12, height - 12, (180, 180, 180))
            break

    return annotated


def _box_color(status, is_selected):
    if not is_selected:
        return (180, 180, 180)
    if status == STATUS_SELECTED:
        return (40, 220, 40)
    if status == STATUS_LOW_CONFIDENCE:
        return (0, 165, 255)
    return (80, 80, 220)


def _clip_box(bbox_xyxy, image_width, image_height):
    x1, y1, x2, y2 = bbox_xyxy
    return (
        int(round(max(0.0, min(float(image_width - 1), x1)))),
        int(round(max(0.0, min(float(image_height - 1), y1)))),
        int(round(max(0.0, min(float(image_width - 1), x2)))),
        int(round(max(0.0, min(float(image_height - 1), y2)))),
    )


def _draw_banner(image, text, color):
    cv2.rectangle(image, (0, 0), (image.shape[1], 34), color, -1)
    cv2.putText(
        image,
        text,
        (10, 23),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )


def _draw_label(image, text, x, y, color):
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.5
    thickness = 1
    (text_width, text_height), baseline = cv2.getTextSize(text, font, scale, thickness)
    x = max(0, min(image.shape[1] - text_width - 4, int(x)))
    y = max(text_height + 4, min(image.shape[0] - 2, int(y)))
    cv2.rectangle(
        image,
        (x, y - text_height - baseline - 4),
        (x + text_width + 4, y + baseline),
        color,
        -1,
    )
    cv2.putText(
        image,
        text,
        (x + 2, y - 3),
        font,
        scale,
        (255, 255, 255),
        thickness,
        cv2.LINE_AA,
    )
