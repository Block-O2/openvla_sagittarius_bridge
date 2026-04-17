#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Compatibility wrapper for the old color_classification.py entry point.

The implementation has moved to language_guided_grasp.py because this node now
does text-conditioned object grounding and grasping, not HSV color classification.
"""

from language_guided_grasp import GroundingDINOGraspNode, LanguageGuidedGraspNode, main


if __name__ == "__main__":
    main()
