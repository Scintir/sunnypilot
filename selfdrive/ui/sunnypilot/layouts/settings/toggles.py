"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import UnknownKeyName
from openpilot.selfdrive.ui.layouts.settings.toggles import TogglesLayout
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp, option_item_sp


class TogglesLayoutSP(TogglesLayout):
  """Toggles panel with EV-limiter + log-upload additions appended after the core toggles."""

  def __init__(self):
    super().__init__()

    # On release/staging branches the prebuilt params_pyx.so has a compiled-
    # in allowlist of param keys. Our custom keys added in params_keys.h
    # aren't in that allowlist until the .so is rebuilt, and every toggle/
    # option widget below reads its param at construction — which would
    # raise UnknownKeyName and break the whole Toggles panel. Preflight one
    # key; if it isn't registered, skip adding the custom widgets.
    try:
      self._params.get_bool("EVLimiterEnabled")
    except UnknownKeyName:
      return

    self._log_upload_toggle = toggle_item_sp(
      title=lambda: tr("Upload CAN logs"),
      description=tr(
        "Rsync completed route logs to your server while offroad and on WiFi. "
        "Set LogUploadDestination via SSH (e.g. user@host:/path/) and place "
        "your SSH private key at /data/scintir/id_ed25519."
      ),
      param="LogUploadEnabled",
    )

    self._ev_limiter_toggle = toggle_item_sp(
      title=lambda: tr("EV Power Limiter"),
      description=tr(
        "Hold the commanded cruise set speed within a small gap of the actual "
        "vehicle speed so propulsion power demand stays inside the EV envelope. "
        "Driver wheel presses adjust the underlying user target in the background; "
        "pedal or brake override as usual."
      ),
      param="EVLimiterEnabled",
    )

    self._ev_power_threshold_kw = option_item_sp(
      title=tr("EV Limiter power threshold (kW)"),
      param="EVLimiterPowerThresholdKW",
      min_value=5, max_value=60, value_change_step=1, inline=True,
    )

    self._ev_dte_floor = option_item_sp(
      title=tr("EV Limiter DTE floor (cluster raw)"),
      param="EVLimiterDTEFloor",
      min_value=1, max_value=50, value_change_step=1, inline=True,
    )

    self._ev_max_gap = option_item_sp(
      title=tr("EV Limiter max gap (mph)"),
      param="EVLimiterMaxGapMph",
      min_value=2, max_value=15, value_change_step=1, inline=True,
    )

    for widget in (
      self._log_upload_toggle,
      self._ev_limiter_toggle,
      self._ev_power_threshold_kw,
      self._ev_dte_floor,
      self._ev_max_gap,
    ):
      self._scroller.add_widget(widget)
