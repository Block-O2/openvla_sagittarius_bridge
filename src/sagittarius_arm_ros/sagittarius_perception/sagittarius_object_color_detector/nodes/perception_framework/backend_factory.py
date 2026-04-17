#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from perception_framework.backends.base import BackendConfig
from perception_framework.backends.grounding_dino import GroundingDinoBackend


def create_backend(config: BackendConfig):
    backend_name = config.name.strip().lower()
    if backend_name in ("grounding_dino", "groundingdino", "gdino"):
        return GroundingDinoBackend(config)
    raise ValueError("Unsupported perception backend: {}".format(config.name))

