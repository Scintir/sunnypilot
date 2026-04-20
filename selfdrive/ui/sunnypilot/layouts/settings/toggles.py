"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.layouts.settings.toggles import TogglesLayout
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp, option_item_sp


class TogglesLayoutSP(TogglesLayout):
  """Toggles panel with Scintir research additions appended after the core toggles."""

  def __init__(self):
    super().__init__()

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
        "Reduce cruise set speed to keep the vehicle in EV mode while the battery "
        "has charge. Commands are only sent while sunnypilot / MADS is engaged; "
        "the driver always overrides via brake or accelerator pedal."
      ),
      param="ScintirEVLimiterEnabled",
    )

    self._scintir_power_threshold = option_item_sp(
      title=tr("Scintir: EV Limiter power threshold (A)"),
      param="ScintirEVLimiterPowerThreshold",
      min_value=10, max_value=80, value_change_step=5, inline=True,
    )

    self._scintir_soc_floor = option_item_sp(
      title=tr("Scintir: EV Limiter battery SOC floor (%)"),
      param="ScintirEVLimiterSOCFloor",
      min_value=5, max_value=50, value_change_step=5, inline=True,
    )

    self._scintir_min_speed = option_item_sp(
      title=tr("Scintir: EV Limiter minimum speed"),
      param="ScintirEVLimiterMinSpeed",
      min_value=5, max_value=40, value_change_step=5, inline=True,
    )

    for widget in (
      self._scintir_rsync_toggle,
      self._scintir_limiter_toggle,
      self._scintir_power_threshold,
      self._scintir_soc_floor,
      self._scintir_min_speed,
    ):
      self._scroller.add_widget(widget)
