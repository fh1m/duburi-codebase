"""
Blackboard utilities — thin helpers over the YASMIN Blackboard API.

YASMIN's Blackboard.get(key) does NOT accept a default argument.
These helpers provide safe access with defaults so states stay clean.
"""

from __future__ import annotations

from yasmin import Blackboard


def bb_get(blackboard: Blackboard, key: str, default=None):
    """Return blackboard[key] if it exists, else *default*."""
    if key in blackboard:
        return blackboard[key]
    return default
