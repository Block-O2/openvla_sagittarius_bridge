#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time

import cv2

from perception_framework.backends.base import BackendConfig, BasePerceptionBackend
from perception_framework.detection_types import DetectionBox, DetectionResult


class GroundingDinoBackend(BasePerceptionBackend):
    """GroundingDINO adapter using the official model/build utilities."""

    source_model = "grounding_dino"

    def __init__(self, config: BackendConfig):
        super().__init__(config)
        self._load_runtime()
        self._load_model()

    def _load_runtime(self):
        if not self.config.model_config or not self.config.model_weights:
            raise RuntimeError(
                "groundingdino_config and groundingdino_weights must be set"
            )
        if not os.path.isfile(self.config.model_config):
            raise RuntimeError(
                "GroundingDINO config file not found: {}".format(
                    self.config.model_config
                )
            )
        if not os.path.isfile(self.config.model_weights):
            raise RuntimeError(
                "GroundingDINO weights file not found: {}".format(
                    self.config.model_weights
                )
            )

        try:
            import torch
            from PIL import Image as PILImage
            import groundingdino.datasets.transforms as T
            from groundingdino.models import build_model
            from groundingdino.util.misc import clean_state_dict
            from groundingdino.util.slconfig import SLConfig
            from groundingdino.util.utils import get_phrases_from_posmap
        except ImportError as exc:
            raise RuntimeError(
                "Failed to import GroundingDINO runtime. Please install the "
                "official GroundingDINO Python package. Original error: {}".format(
                    exc
                )
            )

        resolved_device = self.config.device
        if resolved_device.startswith("cuda") and not torch.cuda.is_available():
            resolved_device = "cpu"

        self.torch = torch
        self.pil_image_cls = PILImage
        self.build_model = build_model
        self.clean_state_dict = clean_state_dict
        self.slconfig = SLConfig
        self.get_phrases_from_posmap = get_phrases_from_posmap
        self.device = resolved_device
        self.transform = T.Compose(
            [
                T.RandomResize([800], max_size=1333),
                T.ToTensor(),
                T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ]
        )

    def _load_model(self):
        args = self.slconfig.fromfile(self.config.model_config)
        args.device = self.device
        model = self.build_model(args)
        checkpoint = self.torch.load(self.config.model_weights, map_location="cpu")
        model.load_state_dict(
            self.clean_state_dict(checkpoint["model"]),
            strict=False,
        )
        self.model = model.eval().to(self.device)

    def infer(self, image_bgr, text_prompt: str) -> DetectionResult:
        image_height, image_width = image_bgr.shape[:2]
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        image_pil = self.pil_image_cls.fromarray(image_rgb)
        image_tensor, _ = self.transform(image_pil, None)

        boxes, scores, phrases = self._predict(image_tensor, text_prompt)
        detection_boxes = []
        for index, box in enumerate(boxes):
            label = phrases[index] if index < len(phrases) else text_prompt
            score = float(scores[index].item()) if hasattr(scores[index], "item") else float(scores[index])
            detection_boxes.append(
                DetectionBox(
                    bbox_xyxy=self._cxcywh_to_xyxy(box, image_width, image_height),
                    score=score,
                    label=label,
                    metadata={"prompt": text_prompt},
                )
            )

        return DetectionResult(
            source_model=self.source_model,
            timestamp=time.time(),
            image_size=(image_width, image_height),
            boxes=detection_boxes,
            metadata={
                "prompt": text_prompt,
                "device": self.device,
                "box_threshold": self.config.box_threshold,
                "text_threshold": self.config.text_threshold,
            },
        )

    def _predict(self, image_tensor, caption):
        caption = self._preprocess_caption(caption)
        image_tensor = image_tensor.to(self.device)

        with self.torch.no_grad():
            outputs = self.model(image_tensor[None], captions=[caption])

        prediction_logits = outputs["pred_logits"].cpu().sigmoid()[0]
        prediction_boxes = outputs["pred_boxes"].cpu()[0]

        mask = prediction_logits.max(dim=1)[0] > self.config.box_threshold
        logits = prediction_logits[mask]
        boxes = prediction_boxes[mask]

        tokenized = self.model.tokenizer(caption)
        phrases = [
            self.get_phrases_from_posmap(
                logit > self.config.text_threshold,
                tokenized,
                self.model.tokenizer,
            ).replace(".", "")
            for logit in logits
        ]
        return boxes, logits.max(dim=1)[0], phrases

    def _preprocess_caption(self, caption):
        result = caption.lower().strip()
        return result if result.endswith(".") else result + "."

    def _cxcywh_to_xyxy(self, box, image_width, image_height):
        cx, cy, width, height = [float(value) for value in box.tolist()]
        x1 = max(0.0, (cx - width / 2.0) * image_width)
        y1 = max(0.0, (cy - height / 2.0) * image_height)
        x2 = min(float(image_width), (cx + width / 2.0) * image_width)
        y2 = min(float(image_height), (cy + height / 2.0) * image_height)
        return x1, y1, x2, y2

