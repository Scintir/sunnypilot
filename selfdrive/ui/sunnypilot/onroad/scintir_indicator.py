"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Small onroad overlay that appears when the Scintir EV power limiter is
actively biasing the stock SCC set speed. Reads state from the carStateSP
message which is published each frame by CarStateExt (one-frame lag from
CarController-side CLU11 TX).
"""
import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


FONT_SIZE = 40
PAD_X = 18
PAD_Y = 10
# Top-right of the usable area; the set-speed MAX box is at top-left so this
# should not collide. Tweak if overlaps appear on specific devices.
MARGIN_X = 40
MARGIN_Y = 220


class ScintirLimiterIndicator(Widget):
  def __init__(self):
    super().__init__()
    self._font = gui_app.font(FontWeight.BOLD)

  def _render(self, rect: rl.Rectangle) -> None:
    sm = ui_state.sm
    try:
      cs_sp = sm["carStateSP"]
    except Exception:
      return

    active = bool(getattr(cs_sp, "scintirEvLimiterActive", False))
    if not active:
      return

    offset = float(getattr(cs_sp, "scintirEvLimiterSetSpeedOffset", 0.0))
    label = f"EV LIMIT  -{int(round(offset))}" if offset >= 0.5 else "EV LIMIT"
    size = measure_text_cached(self._font, label, FONT_SIZE)

    box_w = size.x + PAD_X * 2
    box_h = size.y + PAD_Y * 2
    box = rl.Rectangle(
      rect.x + rect.width - box_w - MARGIN_X,
      rect.y + MARGIN_Y,
      box_w,
      box_h,
    )

    rl.draw_rectangle_rounded(box, 0.25, 4, rl.Color(0xff, 0x8c, 0x00, 0xdc))
    rl.draw_text_ex(
      self._font,
      label,
      rl.Vector2(box.x + PAD_X, box.y + PAD_Y),
      FONT_SIZE,
      0,
      rl.WHITE,
    )
