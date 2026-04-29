"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Onroad overlay for the EV power limiter. Always-on banner once CC has been
engaged, sized for the comma 4 (mici) 536×240 landscape display.

Three lines, anchored top-right next to the cluster set-speed circle (which
lives top-left at ~(21,14)→(183,176)):

   User 65 mph          <- driver's stored target (carStateSP.evLimiterUserTargetSpeed)
   PWR  18 / 40 kW      <- est_power_w / EVLimiterPowerThresholdKW (carStateSP.estPowerW + param)
   STATE: LIMITING      <- limiter state (carStateSP.evLimiterState)

Iter6 changes (drive #5 retro):
- Dropped the SET line (cluster set is already shown on the instrument
  cluster — redundant).
- Added PWR line so user can compare against the cluster gauge and notice
  if the estimator is broken (drive #5: gradeAccel was silently 0 for
  173k samples and we'd have caught it sooner with HUD visibility).
- Renamed "EV TGT" → "User" per user clarification request.
- Box dimensions unchanged (still three lines, same fonts) — User+PWR are
  the same height as the dropped TGT+SET were.

Color codes the banner border by state:
  green  - IDLE
  amber  - SOFT_CAP_ACTIVE / LIMITING
  blue   - RECOVERY_ACTIVE / RECOVERING
  grey   - DRIVER_OVERRIDE_*
  dark   - DISABLED / STANDSTILL
PWR text is colored by power-vs-threshold ratio: green <80%, amber 80-100%,
red >100%. So the user can glance and see "am I in the danger zone."
"""
import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


FONT_SIZE_BIG = 24     # for User XX mph (was 56 — too big for mici 240 px-tall screen)
FONT_SIZE_SMALL = 16   # for PWR + STATE labels
PAD_X = 12
PAD_Y = 6
LINE_GAP = 2
BORDER_PX = 2
RIGHT_MARGIN = 14      # distance from screen right edge
TOP_MARGIN = 14        # distance from screen top edge

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

# Power text color by current/threshold ratio
PWR_COLOR_GREEN = rl.Color(0x40, 0xc0, 0x40, 0xff)
PWR_COLOR_AMBER = rl.Color(0xff, 0xc0, 0x40, 0xff)
PWR_COLOR_RED = rl.Color(0xff, 0x60, 0x40, 0xff)


def _power_color(est_kw: float, threshold_kw: float) -> rl.Color:
  if threshold_kw <= 0:
    return PWR_COLOR_GREEN
  ratio = est_kw / threshold_kw
  if ratio < 0.80:
    return PWR_COLOR_GREEN
  if ratio < 1.00:
    return PWR_COLOR_AMBER
  return PWR_COLOR_RED


def _read_power_threshold_kw() -> int:
  """Read EVLimiterPowerThresholdKW (default 40) for the PWR line. Wrapped
  to survive UnknownKeyName on a stale params_pyx.so."""
  try:
    from openpilot.common.params import Params
    raw = Params().get("EVLimiterPowerThresholdKW")
    if raw is None:
      return 40
    return int(raw)
  except Exception:
    return 40


class EVLimiterIndicator(Widget):
  def __init__(self):
    super().__init__()
    self._font_big = gui_app.font(FontWeight.BOLD)
    self._font_small = gui_app.font(FontWeight.MEDIUM)
    # Cache the power threshold; re-read every ~1 s to pick up param changes
    # without paying a Params hit every frame at 60 Hz.
    self._power_threshold_kw = _read_power_threshold_kw()
    self._threshold_refresh_counter = 0

  def _render(self, rect: rl.Rectangle) -> None:
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

    state = int(getattr(cs_sp, "evLimiterState", 7))
    user_target_ms = float(getattr(cs_sp, "evLimiterUserTargetSpeed", 0.0))
    est_power_w = float(getattr(cs_sp, "estPowerW", 0.0))

    # Don't draw at all if the limiter has never engaged on this trip
    # (user_target_ms == 0 means engage rising edge hasn't fired).
    if state == 7 and user_target_ms <= 0.5:
      return

    # Refresh power-threshold param at ~1 Hz (60 frames at 60 fps)
    self._threshold_refresh_counter = (self._threshold_refresh_counter + 1) % 60
    if self._threshold_refresh_counter == 0:
      self._power_threshold_kw = _read_power_threshold_kw()

    if ui_state.is_metric:
      conv = 3.6
      unit = "kph"
    else:
      conv = 2.23693629
      unit = "mph"

    target_disp = int(round(user_target_ms * conv)) if user_target_ms > 0.5 else 0
    est_kw = max(0.0, est_power_w) / 1000.0
    threshold_kw = self._power_threshold_kw

    user_str = f"User {target_disp} {unit}" if target_disp > 0 else f"User -- {unit}"
    pwr_str = f"PWR {int(round(est_kw))} / {threshold_kw} kW"
    state_str = f"STATE: {STATE_NAMES.get(state, str(state))}"
    border_color = STATE_COLORS.get(state, STATE_COLORS[0])
    pwr_color = _power_color(est_kw, threshold_kw)

    # Measure all three to size the banner
    s_user = measure_text_cached(self._font_big, user_str, FONT_SIZE_BIG)
    s_pwr = measure_text_cached(self._font_small, pwr_str, FONT_SIZE_SMALL)
    s_state = measure_text_cached(self._font_small, state_str, FONT_SIZE_SMALL)

    content_w = max(s_user.x, s_pwr.x, s_state.x)
    box_w = content_w + PAD_X * 2
    content_h = s_user.y + s_pwr.y + s_state.y + LINE_GAP * 2
    box_h = content_h + PAD_Y * 2

    # Anchor top-right next to the cluster set-speed circle (cluster lives
    # top-left). Hard-clamp into rect so an unexpectedly small display can
    # never push the box off-screen — drive #4 hit this exactly: 240 px
    # tall display + TOP_MARGIN=200 + 140 px box = 100 px clipped.
    box_x = rect.x + rect.width - box_w - RIGHT_MARGIN
    box_y = rect.y + TOP_MARGIN
    box_x = max(rect.x, min(box_x, rect.x + rect.width - box_w))
    box_y = max(rect.y, min(box_y, rect.y + rect.height - box_h))
    box = rl.Rectangle(box_x, box_y, box_w, box_h)

    # Background (semi-transparent dark) + colored border framing
    rl.draw_rectangle_rounded(box, 0.18, 10, rl.Color(0x10, 0x10, 0x14, 0xcc))
    rl.draw_rectangle_rounded_lines_ex(box, 0.18, 10, BORDER_PX, border_color)

    y = box.y + PAD_Y
    rl.draw_text_ex(self._font_big, user_str,
                    rl.Vector2(box.x + (box_w - s_user.x) / 2, y),
                    FONT_SIZE_BIG, 0, rl.WHITE)
    y += s_user.y + LINE_GAP
    rl.draw_text_ex(self._font_small, pwr_str,
                    rl.Vector2(box.x + (box_w - s_pwr.x) / 2, y),
                    FONT_SIZE_SMALL, 0, pwr_color)
    y += s_pwr.y + LINE_GAP
    rl.draw_text_ex(self._font_small, state_str,
                    rl.Vector2(box.x + (box_w - s_state.x) / 2, y),
                    FONT_SIZE_SMALL, 0, border_color)
