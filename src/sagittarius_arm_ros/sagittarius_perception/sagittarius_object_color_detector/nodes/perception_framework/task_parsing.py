#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
from dataclasses import dataclass, field
from typing import List, Optional


CHINESE_PATTERNS = (
    re.compile(
        r"^\s*(?:请)?(?:把|将)?\s*(?P<pick>.+?)\s*(?:抓起|抓取|抓住|拿起|拾取|夹起|抓)\s*(?:后)?\s*(?:放到|放进|放入|放在)\s*(?P<place>.+?)\s*(?:里面|里|中)?\s*$"
    ),
    re.compile(
        r"^\s*(?:请)?(?:把|将)?\s*(?P<pick>.+?)\s*(?:放到|放进|放入|放在)\s*(?P<place>.+?)\s*(?:里面|里|中)?\s*$"
    ),
    re.compile(
        r"^\s*(?:请)?(?:抓起|抓取|抓住|拿起|拾取|夹起|抓)\s*(?P<pick>.+?)\s*(?:后)?\s*(?:放到|放进|放入|放在)\s*(?P<place>.+?)\s*(?:里面|里|中)?\s*$"
    ),
)

ENGLISH_PATTERNS = (
    re.compile(
        r"^\s*(?:please\s+)?(?:pick(?:\s+up)?|grab)\s+(?P<pick>.+?)\s+(?:and\s+)?(?:place|put|drop)\s+(?:it\s+)?(?:into|in|inside|onto|on)\s+(?P<place>.+?)\s*$",
        re.IGNORECASE,
    ),
    re.compile(
        r"^\s*(?:please\s+)?(?:place|put|drop)\s+(?P<pick>.+?)\s+(?:to\s+|on\s+)?(?P<place>(?:the\s+)?(?:left|right)\s+of\s+.+?)\s*$",
        re.IGNORECASE,
    ),
    re.compile(
        r"^\s*(?P<pick>.+?)\s+(?P<place>(?:the\s+)?(?:left|right)\s+of\s+.+?)\s*$",
        re.IGNORECASE,
    ),
    re.compile(
        r"^\s*(?P<pick>.+?)\s+(?:to|into|onto|on)\s+(?P<place>.+?)\s*$",
        re.IGNORECASE,
    ),
)

MULTI_STEP_SPLITTER = re.compile(
    r"\s*(?:，|,|；|;|然后|再把|再将|并且|并| and then )\s*"
)
LETTER_TARGET = re.compile(
    r"^(?:字母\s*)?([A-Da-d])(?:\s*(?:点|位置|区域|区|格|处))?$"
)
RELATIVE_PLACE_PATTERNS = (
    re.compile(
        r"^(?:the\s+)?(?P<direction>left|right)\s+of\s+(?P<reference>.+?)$",
        re.IGNORECASE,
    ),
    re.compile(
        r"^(?P<reference>.+?)\s*(?P<direction>左边|左侧|左面|右边|右侧|右面)$"
    ),
)


@dataclass
class TaskStep:
    pick_target_text: str
    place_target_text: Optional[str] = None
    place_relation: Optional[str] = None
    place_reference_text: Optional[str] = None

    @property
    def is_pick_and_place(self) -> bool:
        return bool(self.place_target_text or self.place_reference_text)


@dataclass
class TaskCommand:
    raw_text: str
    steps: List[TaskStep] = field(default_factory=list)

    @property
    def pick_target_text(self) -> str:
        return self.steps[0].pick_target_text if self.steps else ""

    @property
    def place_target_text(self) -> Optional[str]:
        return self.steps[0].place_target_text if self.steps else None

    @property
    def is_pick_and_place(self) -> bool:
        return bool(self.steps and self.steps[0].is_pick_and_place)


def parse_task_command(text: str) -> TaskCommand:
    normalized = _normalize_phrase(text)
    steps = []
    for segment in _split_multi_step_segments(normalized):
        step = _parse_single_step(segment)
        if step is not None:
            steps.append(step)

    if steps:
        return TaskCommand(raw_text=normalized, steps=steps)

    fallback = TaskStep(pick_target_text=normalized)
    return TaskCommand(raw_text=normalized, steps=[fallback])


def _split_multi_step_segments(text: str) -> List[str]:
    text = _normalize_phrase(text)
    if not text:
        return []
    segments = [segment for segment in MULTI_STEP_SPLITTER.split(text) if segment]
    return segments or [text]


def _parse_single_step(text: str) -> Optional[TaskStep]:
    normalized = _normalize_phrase(text)
    if not normalized:
        return None

    for pattern in CHINESE_PATTERNS + ENGLISH_PATTERNS:
        match = pattern.match(normalized)
        if not match:
            continue
        pick = _strip_pick_prefix(_normalize_phrase(match.group("pick")))
        place = _normalize_phrase(match.group("place"))
        place_target, place_relation, place_reference = _parse_place_target(place)
        if pick and (place_target or place_reference):
            return TaskStep(
                pick_target_text=pick,
                place_target_text=place_target,
                place_relation=place_relation,
                place_reference_text=place_reference,
            )
    return TaskStep(pick_target_text=normalized)


def _normalize_phrase(text: str) -> str:
    cleaned = (text or "").strip()
    cleaned = cleaned.strip(".,;:!?，。；：！？")
    cleaned = re.sub(r"\s+", " ", cleaned)
    return cleaned


def _strip_pick_prefix(text: str) -> str:
    cleaned = _normalize_phrase(text)
    cleaned = re.sub(
        r"^(?:请|把|将|抓起|抓取|抓住|拿起|拾取|夹起|抓)\s*",
        "",
        cleaned,
        flags=re.IGNORECASE,
    )
    return _normalize_phrase(cleaned)


def _normalize_place_target(text: str) -> str:
    cleaned = _normalize_phrase(text)
    match = LETTER_TARGET.match(cleaned)
    if match:
        return "letter {}".format(match.group(1).upper())
    return cleaned


def _parse_place_target(text: str):
    cleaned = _normalize_phrase(text)
    for pattern in RELATIVE_PLACE_PATTERNS:
        match = pattern.match(cleaned)
        if not match:
            continue
        direction = match.group("direction").lower()
        reference = _normalize_place_target(_normalize_phrase(match.group("reference")))
        if direction.startswith("左") or direction == "left":
            return None, "left_of", reference
        if direction.startswith("右") or direction == "right":
            return None, "right_of", reference
    return _normalize_place_target(cleaned), None, None
