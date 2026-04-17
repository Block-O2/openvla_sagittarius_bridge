#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import yaml


class VisionPlaneMapper:
    """Maps image pixel centers to the calibrated Sagittarius grasp plane."""

    def __init__(self, vision_config_path: str):
        self.params = self._load_vision_config(vision_config_path)

    def map_pixel_center(self, center):
        pixel_x, pixel_y = center
        grasp_x = self.params["k1"] * pixel_y + self.params["b1"]
        grasp_y = self.params["k2"] * pixel_x + self.params["b2"]
        return grasp_x, grasp_y

    def _load_vision_config(self, filename):
        with open(filename, "r") as stream:
            content = yaml.safe_load(stream)
        return {
            "k1": float(content["LinearRegression"]["k1"]),
            "b1": float(content["LinearRegression"]["b1"]),
            "k2": float(content["LinearRegression"]["k2"]),
            "b2": float(content["LinearRegression"]["b2"]),
        }

