"""SimLoop — 원본 세 스크립트가 복제하던 while 루프 하나.

매 프레임: app.update() → viewer.tick() → (재생 중이면) on_step(frame) → 주기 로그.
타임라인이 멈췄다가 다시 재생되면 stop/play 를 한 번 돌려 물리 상태를 리셋한다 (원본 reset_needed 패턴).
"""
from __future__ import annotations

from typing import Callable, Optional

RESET_SETTLE_STEPS = 5
READY_WARMUP_STEPS = 10


def play_and_warmup(steps: int = READY_WARMUP_STEPS) -> None:
    import isaacsim.core.experimental.utils.app as app_utils

    app_utils.play()
    app_utils.update_app(steps=steps)


def current_sim_time() -> float:
    import omni.timeline

    return float(omni.timeline.get_timeline_interface().get_current_time())


class SimLoop:
    def __init__(self, app, on_step: Callable[[int], None], viewer=None, log_every: int = 300,
                 log_fn: Optional[Callable[[int], None]] = None) -> None:
        self.app = app
        self.on_step = on_step
        self.viewer = viewer
        self.log_every = log_every
        self.log_fn = log_fn
        self.frame = 0

    def _reset_playback(self, app_utils) -> None:
        app_utils.stop()
        app_utils.update_app(steps=RESET_SETTLE_STEPS)
        app_utils.play()
        app_utils.update_app(steps=RESET_SETTLE_STEPS)

    def run(self) -> None:
        import isaacsim.core.experimental.utils.app as app_utils

        reset_needed = False
        while self.app.is_running():
            self.app.update()
            if self.viewer is not None:
                self.viewer.tick()
            self.app.disable_viewport_updates()
            self.frame += 1
            if self.log_fn and (self.frame == 1 or self.frame % self.log_every == 0):
                self.log_fn(self.frame)
            if not app_utils.is_playing():
                reset_needed = True
                continue
            if reset_needed:
                self._reset_playback(app_utils)
                reset_needed = False
            self.on_step(self.frame)
        app_utils.stop()
