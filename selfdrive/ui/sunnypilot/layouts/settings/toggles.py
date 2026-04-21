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
  """Toggles panel with Scintir research additions appended after the core toggles."""

  def __init__(self):
    super().__init__()

    # On release/staging branches the prebuilt params_pyx.so has a compiled-
    # in allowlist of param keys. Scintir* keys added in params_keys.h won't
    # exist in that allowlist until the .so is rebuilt, and every toggle/
    # option widget below reads its param at construction — which would
    # raise UnknownKeyName and break the whole Toggles panel. Preflight one
    # Scintir key; if it isn't registered, skip adding the Scintir widgets.
    try:
      self._params.get_bool("ScintirEVLimiterEnabled")
    except UnknownKeyName:
      return

    self._scintir_rsync_toggle = toggle_item_sp(
      title=lambda: tr("Scintir: Upload CAN logs"),
      description=tr(
        "Rsync completed route logs to your server while offroad and on WiFi. "
        "Set ScintirRsyncDestination via SSH (e.g. user@host:/path/) and place "
        "your SSH private key at /data/scintir/id_ed25519."
      ),
      param="ScintirRsyncEnabled",
    )

    self._scintir_limiter_toggle = toggle_item_sp(
      title=lambda: tr("Scintir: EV Power Limiter"),
      description=tr(
        "Reduce cruise set speed when propulsion-power demand looks about to "
        "trigger ICE engagement. Uses aBasis (aggregated driver+SCC accel) x "
        "vEgo as the demand estimate. Commands are only sent while sunnypilot "
        "/ MADS is engaged; the driver always overrides via brake or accelerator pedal."
      ),
      param="ScintirEVLimiterEnabled",
    )

    self._scintir_power_threshold_kw = option_item_sp(
      title=tr("Scintir: EV Limiter power threshold (kW)"),
      param="ScintirEVLimiterPowerThresholdKW",
      min_value=5, max_value=60, value_change_step=1, inline=True,
    )

    self._scintir_dte_floor = option_item_sp(
      title=tr("Scintir: EV Limiter DTE floor (cluster raw)"),
      param="ScintirEVLimiterDTEFloor",
      min_value=1, max_value=50, value_change_step=1, inline=True,
    )

    self._scintir_min_speed = option_item_sp(
      title=tr("Scintir: EV Limiter minimum speed"),
      param="ScintirEVLimiterMinSpeed",
      min_value=5, max_value=40, value_change_step=5, inline=True,
    )

    for widget in (
      self._scintir_rsync_toggle,
      self._scintir_limiter_toggle,
      self._scintir_power_threshold_kw,
      self._scintir_dte_floor,
      self._scintir_min_speed,
    ):
      self._scroller.add_widget(widget)
