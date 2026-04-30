"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Tap-to-cycle integer setting for the Comma 4 (mici) UI. Used as a numeric
slider substitute in the mici Toggles panel since the upstream mici button
library only ships BigParamControl (bool) and BigMultiParamToggle (enum).
"""
from openpilot.common.params import Params, UnknownKeyName
from openpilot.selfdrive.ui.mici.widgets.button import BigButton


class CyclingIntButton(BigButton):
  """Tap-to-cycle integer parameter button.

  Each tap advances the param value to the next entry in `values` and
  wraps around at the end. The current value is shown as the button's
  value text (via BigButton.set_value).

  Resilient to missing param keys: if the prebuilt params_pyx.so doesn't
  know the key yet (UnknownKeyName), the button shows "--" and taps are
  no-ops so the rest of the Toggles panel keeps working.
  """

  def __init__(self, title: str, param: str, values: list[int], suffix: str = "", default: int | None = None):
    super().__init__(title, "")
    self._param = param
    self._values = list(values)
    self._suffix = suffix
    self._default = default if default is not None else self._values[0]
    self._params = Params()
    self._reachable = True
    self._refresh_display()

  def _current(self) -> int:
    try:
      raw = self._params.get(self._param, return_default=True)
      self._reachable = True
      if raw is None:
        return self._default
      return int(raw)
    except (ValueError, TypeError):
      return self._default
    except UnknownKeyName:
      self._reachable = False
      return self._default

  def _refresh_display(self) -> None:
    if not self._reachable:
      self.set_value("--")
      return
    cur = self._current()
    if self._reachable:
      self.set_value(f"{cur}{self._suffix}")
    else:
      self.set_value("--")

  def _handle_mouse_release(self, mouse_pos):
    super()._handle_mouse_release(mouse_pos)
    if not self._reachable:
      return
    cur = self._current()
    if not self._reachable:
      return
    # Find nearest discrete step (tolerant of out-of-set stored values).
    if cur in self._values:
      idx = self._values.index(cur)
    else:
      idx = min(range(len(self._values)), key=lambda i: abs(self._values[i] - cur))
    nxt = self._values[(idx + 1) % len(self._values)]
    # Drive #7 UI crash root cause: passing `str(nxt)` to an INT-typed param
    # raises TypeError in params_pyx (proposed_type=str, expected_type=INT).
    # The exception propagated, killing the UI process. Fix: pass int directly
    # — params_pyx's (int, INT) cast handles the conversion. Catch TypeError
    # too as defense-in-depth so a future param-type mismatch doesn't crash UI.
    try:
      self._params.put(self._param, nxt)
    except UnknownKeyName:
      self._reachable = False
    except TypeError as e:
      print(f"[CyclingIntButton] put({self._param}={nxt}) failed: {e}")
      self._reachable = False
    self._refresh_display()
