"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Onroad overlay for the EV power limiter. Always-on banner once CC has been
engaged, sized for the comma 4 (mici) 536×240 landscape display.

Three lines, anchored top-right next to the cluster set-speed circle (which
lives top-left at ~(21,14)→(183,176)):

   User 65 mph          <- driver's stored target (carStateSP.evLimiterUserTargetSpeed)
   PWR 18 kW            <- est_power_w (filtered, asymmetric LP for stability)
   STATE: LIMITING      <- limiter state (carStateSP.evLimiterState)

Iter7 HUD changes (drive #6 user feedback):
- PWR line uses the same big font as User (24 pt) — was 16 pt small,
  user found it too small to glance at.
- PWR text is white (was green/amber/red ratio coloring) — user found
  green hard to read in daylight.
- Dropped "/40 kW" threshold from the PWR text — user has the slider
  in settings; the live value is what matters for monitoring.
- estPowerW is asymmetric-LP filtered server-side (carstate_ext.py):
  fast-rise τ=0.15 s, slow-fall τ=2 s, published as max(raw, filtered)
  so spikes show immediately but baseline is stable.

Banner border still color-coded by state:
  green  - IDLE
  amber  - SOFT_CAP_ACTIVE / LIMITING
  blue   - RECOVERY_ACTIVE / RECOVERING
  grey   - DRIVER_OVERRIDE_*
  dark   - DISABLED / STANDSTILL
"""
import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


FONT_SIZE_BIG = 24     # for User XX mph + PWR XX kW (iter7: PWR same size as User)
FONT_SIZE_SMALL = 16   # for STATE label only
PAD_X = 12
PAD_Y = 6
LINE_GAP = 2
BORDER_PX = 2
RIGHT_MARGIN = 14      # distance from screen right edge
TOP_MARGIN = 14        # distance from screen top edge

# iter16a (Phase A): live request-direction arrow. Shows whether the comma device
# is asking the SCC set speed to go DOWN (limiting) or UP (recovering) RIGHT NOW,
# and whether the SCC is honoring it. Driven by carStateSP.evLimiterButtonDir
# (actual emitted button, NOT mere intent) + evLimiterRequestHonored.
ARROW_W = 56           # half-width of the triangle base (px)
ARROW_H = 64           # height of the triangle (px)
ARROW_GAP = 12         # gap between arrow and the text banner
# button_dir: 0 NONE, 1 UP(RES), 2 DOWN(SET). honored: 0 unknown, 1 honored, 2 ignored.
ARROW_COLOR_HONORED = rl.Color(0x30, 0xc0, 0x30, 0xff)   # green — SCC followed
ARROW_COLOR_IGNORED = rl.Color(0xff, 0x40, 0x40, 0xff)   # red — comma asked, SCC didn't move
ARROW_COLOR_PENDING = rl.Color(0xf0, 0xf0, 0xf0, 0xff)   # white — asking, not yet resolved

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
    self._blink = 0   # frame counter for the "ignored" flash

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
    button_dir = int(getattr(cs_sp, "evLimiterButtonDir", 0))      # 0 none, 1 up(RES), 2 down(SET)
    request_honored = int(getattr(cs_sp, "evLimiterRequestHonored", 0))  # 0 unknown, 1 honored, 2 ignored

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
    est_kw = max(0.0, est_power_w) / 1000.0

    user_str = f"User {target_disp} {unit}" if target_disp > 0 else f"User -- {unit}"
    pwr_str = f"PWR {int(round(est_kw))} kW"
    state_str = f"STATE: {STATE_NAMES.get(state, str(state))}"
    border_color = STATE_COLORS.get(state, STATE_COLORS[0])

    # Measure all three to size the banner — User + PWR are now both BIG.
    s_user = measure_text_cached(self._font_big, user_str, FONT_SIZE_BIG)
    s_pwr = measure_text_cached(self._font_big, pwr_str, FONT_SIZE_BIG)
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
    rl.draw_text_ex(self._font_big, pwr_str,
                    rl.Vector2(box.x + (box_w - s_pwr.x) / 2, y),
                    FONT_SIZE_BIG, 0, rl.WHITE)
    y += s_pwr.y + LINE_GAP
    rl.draw_text_ex(self._font_small, state_str,
                    rl.Vector2(box.x + (box_w - s_state.x) / 2, y),
                    FONT_SIZE_SMALL, 0, border_color)

    # iter16a: live request-direction arrow, to the LEFT of the banner.
    self._draw_request_arrow(box, button_dir, request_honored)

  def _draw_request_arrow(self, box: rl.Rectangle, button_dir: int, honored: int) -> None:
    """Big ↑/↓ triangle showing the comma device's CURRENT set-speed request.
    Green = SCC honored it, red (flashing) = comma asked but the set speed
    didn't move, white = asking / not yet resolved. Hidden when not asking."""
    self._blink = (self._blink + 1) % 40   # ~0.66 s period at 60 Hz UI
    if button_dir == 0:
      return

    if honored == 1:
      color = ARROW_COLOR_HONORED
    elif honored == 2:
      # Flash the "ignored" arrow so it draws the eye.
      if self._blink >= 20:
        return
      color = ARROW_COLOR_IGNORED
    else:
      color = ARROW_COLOR_PENDING

    cx = box.x - ARROW_GAP - ARROW_W
    cy = box.y + box.height / 2.0
    cx = max(ARROW_W, cx)   # keep the left vertex (cx - ARROW_W) on-screen

    if button_dir == 2:   # DOWN (SET) — apex at bottom
      apex = rl.Vector2(cx, cy + ARROW_H / 2.0)
      left = rl.Vector2(cx - ARROW_W, cy - ARROW_H / 2.0)
      right = rl.Vector2(cx + ARROW_W, cy - ARROW_H / 2.0)
    else:                 # UP (RES) — apex at top
      apex = rl.Vector2(cx, cy - ARROW_H / 2.0)
      left = rl.Vector2(cx - ARROW_W, cy + ARROW_H / 2.0)
      right = rl.Vector2(cx + ARROW_W, cy + ARROW_H / 2.0)
    # pyray winding: draw both windings so the fill always shows regardless of orientation.
    rl.draw_triangle(apex, left, right, color)
    rl.draw_triangle(apex, right, left, color)
