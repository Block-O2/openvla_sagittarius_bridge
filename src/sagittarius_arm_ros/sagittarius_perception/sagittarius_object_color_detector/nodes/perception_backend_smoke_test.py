#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse

import cv2

from perception_framework.backends.base import BackendConfig
from perception_framework.backend_factory import create_backend
from perception_framework.decision import evaluate_target_selection
from perception_framework.visualization import draw_detection_overlay


def main():
    parser = argparse.ArgumentParser(
        description="Run one perception backend inference on a local image."
    )
    parser.add_argument("--image", required=True, help="Path to a local image")
    parser.add_argument("--text", required=True, help="Text target prompt")
    parser.add_argument(
        "--backend",
        default="grounding_dino",
        help="Backend name, for example grounding_dino",
    )
    parser.add_argument("--config", required=True, help="Model config path")
    parser.add_argument("--weights", required=True, help="Model weights path")
    parser.add_argument("--device", default="cpu", help="cuda or cpu")
    parser.add_argument("--box-threshold", type=float, default=0.35)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    parser.add_argument(
        "--min-grasp-score",
        type=float,
        default=0.35,
        help="Score gate used by the grasp decision layer",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Optional path for an annotated output image",
    )
    args = parser.parse_args()

    image = cv2.imread(args.image)
    if image is None:
        raise RuntimeError("Failed to read image: {}".format(args.image))

    backend = create_backend(
        BackendConfig(
            name=args.backend,
            device=args.device,
            box_threshold=args.box_threshold,
            text_threshold=args.text_threshold,
            model_config=args.config,
            model_weights=args.weights,
        )
    )
    result = backend.infer(image, args.text)
    decision = evaluate_target_selection(result, args.text, args.min_grasp_score)
    selected = decision.selected_box

    print("source_model:", result.source_model)
    print("image_size:", result.image_size)
    print("num_boxes:", len(result.boxes))
    print("decision_status:", decision.status)
    print("decision_reason:", decision.reason)
    if selected is None:
        print("selected: None")
        if args.output:
            annotated = draw_detection_overlay(image, result, args.text, decision)
            cv2.imwrite(args.output, annotated)
            print("annotated_output:", args.output)
        return
    print("selected_label:", selected.label)
    print("selected_score:", "{:.4f}".format(selected.score))
    print("selected_bbox_xyxy:", tuple(round(value, 2) for value in selected.bbox_xyxy))
    print("selected_center:", tuple(round(value, 2) for value in selected.center))
    if args.output:
        annotated = draw_detection_overlay(image, result, args.text, decision)
        cv2.imwrite(args.output, annotated)
        print("annotated_output:", args.output)


if __name__ == "__main__":
    main()
