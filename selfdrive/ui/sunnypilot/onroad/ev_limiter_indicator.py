"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Onroad overlay for the EV power limiter. Always-on banner once CC has been
engaged, deliberately positioned and sized so the driver can't miss it
(the previous middle-right placement was reportedly invisible on the road).

Three lines, big font, top-center under the speed display:

   EV TARGET 65            <- driver's stored target (carStateSP.evLimiterUserTargetSpeed)
   SET 55                  <- current cluster set speed (carState.cruiseState.speed)
   STATE: LIMITING         <- limiter state (carStateSP.evLimiterState)

Color codes the banner border:
  green  - IDLE / steady
  amber  - SOFT_CAP_ACTIVE (pulling observed down)
  blue   - RECOVERY_ACTIVE
  grey   - DRIVER_OVERRIDE_*
  dark   - DISABLED / STANDSTILL
"""
import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


FONT_SIZE_BIG = 56     # for TARGET / SET numbers
FONT_SIZE_SMALL = 32   # for STATE label
PAD_X = 22
PAD_Y = 12
LINE_GAP = 6
BORDER_PX = 4
TOP_MARGIN = 200       # below the cluster set-speed circle which lives top-left, but
                        # we anchor center-top here for prominence

STATE_NAMES = {
  0: "IDLE",
  1: "STANDSTILL",
  2: "LIMITING",
  3: "RECOVERING",
  4: "DRV SET",
  5: "DRV RES",
  6: "BUS FAULT",
  7: "DISABLED",
}

STATE_COLORS = {
  0: rl.Color(0x30, 0xa0, 0x30, 0xe6),  # green - idle/steady
  1: rl.Color(0x60, 0x60, 0x60, 0xe6),  # grey - standstill
  2: rl.Color(0xff, 0x8c, 0x00, 0xee),  # amber - limiting
  3: rl.Color(0x28, 0x80, 0xff, 0xee),  # blue - recovering
  4: rl.Color(0x80, 0x80, 0x80, 0xe6),  # grey - driver override
  5: rl.Color(0x80, 0x80, 0x80, 0xe6),
  6: rl.Color(0xff, 0x40, 0x40, 0xe6),  # red - bus fault
  7: rl.Color(0x40, 0x40, 0x40, 0xa0),  # dark - disabled
}


class EVLimiterIndicator(Widget):
  def __init__(self):
    super().__init__()
    self._font_big = gui_app.font(FontWeight.BOLD)
    self._font_small = gui_app.font(FontWeight.MEDIUM)

  def _render(self, rect: rl.Rectangle) -> None:
    try:
      self._render_inner(rect)
    except Exception as e:
      print(f"[ev_limiter_indicator] render suppressed: {type(e).__name__}: {e}")

  def _render_inner(self, rect: rl.Rectangle) -> None:
    sm = ui_state.sm
    try:
      cs_sp = sm["carStateSP"]
      cs = sm["carState"]
    except Exception:
      return

    state = int(getattr(cs_sp, "evLimiterState", 7))
    user_target_ms = float(getattr(cs_sp, "evLimiterUserTargetSpeed", 0.0))
    set_speed_ms = float(cs.cruiseState.speed)

    # Don't draw at all if the limiter has never engaged on this trip
    # (user_target_ms == 0 means engage rising edge hasn't fired).
    if state == 7 and user_target_ms <= 0.5:
      return

    if ui_state.is_metric:
      conv = 3.6
      unit = "kph"
    else:
      conv = 2.23693629
      unit = "mph"

    target_disp = int(round(user_target_ms * conv)) if user_target_ms > 0.5 else 0
    set_disp = int(round(set_speed_ms * conv)) if set_speed_ms > 0.5 else 0

    target_str = f"EV TARGET  {target_disp} {unit}" if target_disp > 0 else f"EV TARGET  -- {unit}"
    set_str = f"SET  {set_disp} {unit}" if set_disp > 0 else f"SET  --"
    state_str = f"STATE: {STATE_NAMES.get(state, str(state))}"
    border_color = STATE_COLORS.get(state, STATE_COLORS[0])

    # Measure all three to size the banner
    s_target = measure_text_cached(self._font_big, target_str, FONT_SIZE_BIG)
    s_set = measure_text_cached(self._font_big, set_str, FONT_SIZE_BIG)
    s_state = measure_text_cached(self._font_small, state_str, FONT_SIZE_SMALL)

    content_w = max(s_target.x, s_set.x, s_state.x)
    box_w = content_w + PAD_X * 2
    content_h = s_target.y + s_set.y + s_state.y + LINE_GAP * 2
    box_h = content_h + PAD_Y * 2

    # Center horizontally, fixed vertical anchor near the top so it sits below
    # the cluster set-speed circle and above the lane preview.
    box = rl.Rectangle(
      rect.x + (rect.width - box_w) / 2,
      rect.y + TOP_MARGIN,
      box_w,
      box_h,
    )

    # Background (semi-transparent dark) + colored border framing
    rl.draw_rectangle_rounded(box, 0.18, 10, rl.Color(0x10, 0x10, 0x14, 0xcc))
    rl.draw_rectangle_rounded_lines_ex(box, 0.18, 10, BORDER_PX, border_color)

    y = box.y + PAD_Y
    rl.draw_text_ex(self._font_big, target_str,
                    rl.Vector2(box.x + (box_w - s_target.x) / 2, y),
                    FONT_SIZE_BIG, 0, rl.WHITE)
    y += s_target.y + LINE_GAP
    rl.draw_text_ex(self._font_big, set_str,
                    rl.Vector2(box.x + (box_w - s_set.x) / 2, y),
                    FONT_SIZE_BIG, 0, rl.WHITE)
    y += s_set.y + LINE_GAP
    rl.draw_text_ex(self._font_small, state_str,
                    rl.Vector2(box.x + (box_w - s_state.x) / 2, y),
                    FONT_SIZE_SMALL, 0, border_color)
