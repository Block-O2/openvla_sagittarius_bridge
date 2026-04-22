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

    def is_degenerate(self, epsilon=1e-9):
        """Return True when pixel changes cannot affect mapped grasp XY."""
        return abs(self.params["k1"]) < epsilon and abs(self.params["k2"]) < epsilon

    def describe(self):
        return "x=k1*pixel_y+b1 (k1={k1:.6f}, b1={b1:.6f}), y=k2*pixel_x+b2 (k2={k2:.6f}, b2={b2:.6f})".format(
            **self.params
        )

    def _load_vision_config(self, filename):
        with open(filename, "r") as stream:
            content = yaml.safe_load(stream)
        return {
            "k1": float(content["LinearRegression"]["k1"]),
            "b1": float(content["LinearRegression"]["b1"]),
            "k2": float(content["LinearRegression"]["k2"]),
            "b2": float(content["LinearRegression"]["b2"]),
        }
