"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Onroad overlay for the EV power limiter. Two lines:
  EV LIMIT  -N mph        (only while the limiter is actively biasing set speed)
  TARGET: NN mph          (whenever the driver has a non-zero stored target)

Second line is always visible once a target is stored — driver asked for
persistent feedback on "what the system will try to recover to." Reads the
carStateSP message (evLimiterActive, evLimiterSetSpeedOffset,
evLimiterUserTargetSpeed) published by CarStateExt.
"""
import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


FONT_SIZE = 40
PAD_X = 18
PAD_Y = 10
LINE_GAP = 8

# Middle-upper right of the usable area. Avoids:
#   - top-left MAX/set-speed circle (y ~ 0..162)
#   - top-center current-speed text (y ~ 90..280)
#   - side blind-spot indicators (y 100..228, x within 128 px of each edge)
#   - bottom-left steering-wheel icon
MARGIN_X = 60
MARGIN_Y = 320


class EVLimiterIndicator(Widget):
  def __init__(self):
    super().__init__()
    self._font = gui_app.font(FontWeight.BOLD)

  def _render(self, rect: rl.Rectangle) -> None:
    # Absolute no-crash policy: any exception here silently no-ops.
    # The indicator is cosmetic; it must never take down the rest of
    # the HUD.
    try:
      self._render_inner(rect)
    except Exception as e:
      print(f"[ev_limiter_indicator] render suppressed: {type(e).__name__}: {e}")

  def _render_inner(self, rect: rl.Rectangle) -> None:
    sm = ui_state.sm
    try:
      cs_sp = sm["carStateSP"]
    except Exception:
      return

    active = bool(getattr(cs_sp, "evLimiterActive", False))
    offset_ms = float(getattr(cs_sp, "evLimiterSetSpeedOffset", 0.0))
    user_target_ms = float(getattr(cs_sp, "evLimiterUserTargetSpeed", 0.0))

    if ui_state.is_metric:
      conv = 3.6
      unit = "kph"
    else:
      conv = 2.23693629
      unit = "mph"

    lines: list[tuple[str, rl.Color]] = []

    if active:
      offset_display = offset_ms * conv
      label = f"EV LIMIT  -{int(round(offset_display))} {unit}" if offset_display >= 0.5 else "EV LIMIT"
      lines.append((label, rl.Color(0xff, 0x8c, 0x00, 0xdc)))

    if user_target_ms > 0.5:
      target_display = user_target_ms * conv
      lines.append((f"TARGET: {int(round(target_display))} {unit}", rl.Color(0x28, 0x80, 0xff, 0xdc)))

    if not lines:
      return

    # Size each line and draw stacked top-to-bottom
    y = rect.y + MARGIN_Y
    for label, bg in lines:
      size = measure_text_cached(self._font, label, FONT_SIZE)
      box_w = size.x + PAD_X * 2
      box_h = size.y + PAD_Y * 2
      box = rl.Rectangle(rect.x + rect.width - box_w - MARGIN_X, y, box_w, box_h)
      rl.draw_rectangle_rounded(box, 0.25, 4, bg)
      rl.draw_text_ex(self._font, label, rl.Vector2(box.x + PAD_X, box.y + PAD_Y), FONT_SIZE, 0, rl.WHITE)
      y += box_h + LINE_GAP
