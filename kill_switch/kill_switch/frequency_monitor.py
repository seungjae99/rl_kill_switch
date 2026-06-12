#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from collections import deque
import time
from typing import Deque


class FrequencyMonitor:
    """Sliding-window frequency monitor for ROS2 sensor callbacks.

    Tracks how often update() is called and triggers when the measured
    frequency falls below min_hz for consecutive_fail consecutive windows.
    """

    def __init__(
        self,
        window_sec: float = 1.0,
        min_hz: float = 3.0,
        consecutive_fail: int = 3,
    ) -> None:
        """
        Args:
            window_sec: Length of the sliding time window in seconds.
            min_hz: Minimum acceptable frequency in Hz.
            consecutive_fail: Number of consecutive under-frequency windows
                              before update() returns True.
        """
        self.window_sec: float = window_sec
        self.min_hz: float = min_hz
        self.consecutive_fail: int = consecutive_fail
        self._ts: Deque[float] = deque()
        self.fail_count: int = 0

    def update(self) -> bool:
        """Record a new message arrival and evaluate frequency.

        Returns:
            True when the trigger condition is met (fail_count >= consecutive_fail).
        """
        now = time.time()
        self._ts.append(now)
        while self._ts and (now - self._ts[0]) > self.window_sec:
            self._ts.popleft()
        hz = len(self._ts) / self.window_sec
        if hz < self.min_hz:
            self.fail_count += 1
        else:
            self.fail_count = 0
        return self.fail_count >= self.consecutive_fail
