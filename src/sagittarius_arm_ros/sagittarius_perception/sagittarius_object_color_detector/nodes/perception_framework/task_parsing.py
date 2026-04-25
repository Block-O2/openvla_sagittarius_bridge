#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
from dataclasses import dataclass
from typing import Optional


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
)


@dataclass
class TaskCommand:
    raw_text: str
    pick_target_text: str
    place_target_text: Optional[str] = None

    @property
    def is_pick_and_place(self) -> bool:
        return bool(self.place_target_text)


def parse_task_command(text: str) -> TaskCommand:
    normalized = _normalize_phrase(text)
    for pattern in CHINESE_PATTERNS + ENGLISH_PATTERNS:
        match = pattern.match(normalized)
        if not match:
            continue
        pick = _strip_pick_prefix(_normalize_phrase(match.group("pick")))
        place = _normalize_phrase(match.group("place"))
        if pick and place:
            return TaskCommand(
                raw_text=normalized,
                pick_target_text=pick,
                place_target_text=place,
            )
    return TaskCommand(raw_text=normalized, pick_target_text=normalized)


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
