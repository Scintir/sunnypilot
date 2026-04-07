from cereal import log

from openpilot.common.params import Params
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigParamControl, BigMultiParamToggle
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.settings.common import restart_needed_callback
from openpilot.selfdrive.ui.ui_state import ui_state


class EVPowerLimitKWButton(BigButton):
  """Cycling button for EV power limit kW setting. Tapping cycles 20->25->...->55->20."""
  KW_VALUES = list(range(20, 60, 5))  # [20, 25, 30, 35, 40, 45, 50, 55]

  def __init__(self):
    super().__init__("EV power limit", "")
    self._params = Params()
    self._load_value()

  def _load_value(self):
    kw = self._params.get("EVPowerLimitKW", return_default=True)
    if kw not in self.KW_VALUES:
      kw = 35
    self.set_subtitle(f"{kw} kW")

  def _handle_mouse_release(self, mouse_pos):
    super()._handle_mouse_release(mouse_pos)
    kw = self._params.get("EVPowerLimitKW", return_default=True)
    if kw not in self.KW_VALUES:
      kw = 35
    idx = self.KW_VALUES.index(kw)
    next_kw = self.KW_VALUES[(idx + 1) % len(self.KW_VALUES)]
    self._params.put("EVPowerLimitKW", next_kw)
    self.set_subtitle(f"{next_kw} kW")

PERSONALITY_TO_INT = log.LongitudinalPersonality.schema.enumerants


class TogglesLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()

    self._personality_toggle = BigMultiParamToggle("driving personality", "LongitudinalPersonality", ["aggressive", "standard", "relaxed"])
    self._experimental_btn = BigParamControl("experimental mode", "ExperimentalMode")
    is_metric_toggle = BigParamControl("use metric units", "IsMetric")
    ldw_toggle = BigParamControl("lane departure warnings", "IsLdwEnabled")
    always_on_dm_toggle = BigParamControl("always-on driver monitor", "AlwaysOnDM")
    record_front = BigParamControl("record & upload driver camera", "RecordFront", toggle_callback=restart_needed_callback)
    record_mic = BigParamControl("record & upload mic audio", "RecordAudio", toggle_callback=restart_needed_callback)
    enable_openpilot = BigParamControl("enable sunnypilot", "OpenpilotEnabledToggle", toggle_callback=restart_needed_callback)

    # EV Power Limit
    self._ev_power_limit_toggle = BigParamControl("EV power limit", "EVPowerLimitEnabled",
                                                   toggle_callback=self._on_ev_power_toggle)
    self._ev_kw_btn = EVPowerLimitKWButton()
    self._ev_power_limit_logging = BigParamControl("EV power limit logging", "EVPowerLimitLogging")

    self._scroller.add_widgets([
      self._personality_toggle,
      self._experimental_btn,
      is_metric_toggle,
      ldw_toggle,
      always_on_dm_toggle,
      record_front,
      record_mic,
      enable_openpilot,
      self._ev_power_limit_toggle,
      self._ev_kw_btn,
      self._ev_power_limit_logging,
    ])

    # Toggle lists
    self._refresh_toggles = (
      ("ExperimentalMode", self._experimental_btn),
      ("IsMetric", is_metric_toggle),
      ("IsLdwEnabled", ldw_toggle),
      ("AlwaysOnDM", always_on_dm_toggle),
      ("RecordFront", record_front),
      ("RecordAudio", record_mic),
      ("OpenpilotEnabledToggle", enable_openpilot),
    )

    enable_openpilot.set_enabled(lambda: not ui_state.engaged)
    record_front.set_enabled(False if ui_state.params.get_bool("RecordFrontLock") else (lambda: not ui_state.engaged))
    record_mic.set_enabled(lambda: not ui_state.engaged)

    if ui_state.params.get_bool("ShowDebugInfo"):
      gui_app.set_show_touches(True)
      gui_app.set_show_fps(True)

    ui_state.add_engaged_transition_callback(self._update_toggles)

  def _update_state(self):
    super()._update_state()

    if ui_state.sm.updated["selfdriveState"]:
      personality = PERSONALITY_TO_INT[ui_state.sm["selfdriveState"].personality]
      if personality != ui_state.personality and ui_state.started:
        self._personality_toggle.set_value(self._personality_toggle._options[personality])
      ui_state.personality = personality

  def _on_ev_power_toggle(self, checked):
    self._ev_kw_btn.set_visible(checked)
    self._ev_power_limit_logging.set_visible(checked)

  def show_event(self):
    super().show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

    # CP gating for experimental mode
    if ui_state.CP is not None:
      if ui_state.has_longitudinal_control:
        self._experimental_btn.set_visible(True)
        self._personality_toggle.set_visible(True)
      else:
        # no long for now
        self._experimental_btn.set_visible(False)
        self._experimental_btn.set_checked(False)
        self._personality_toggle.set_visible(False)
        ui_state.params.remove("ExperimentalMode")

    # Refresh toggles from params to mirror external changes
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))

    # Sync EV power limit visibility
    ev_enabled = ui_state.params.get_bool("EVPowerLimitEnabled")
    self._ev_power_limit_toggle.set_checked(ev_enabled)
    self._ev_kw_btn.set_visible(ev_enabled)
    self._ev_power_limit_logging.set_visible(ev_enabled)
    self._ev_kw_btn._load_value()
