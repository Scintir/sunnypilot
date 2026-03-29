from cereal import log
from openpilot.common.params import Params

from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigParamControl, BigMultiParamToggle
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.settings.common import restart_needed_callback
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog

PERSONALITY_TO_INT = log.LongitudinalPersonality.schema.enumerants


class BigIntParamSelect(BigButton):
  def __init__(self, text: str, param: str, values: list[int], value_formatter=None):
    super().__init__(text, "")
    self._params = Params()
    self._param = param
    self._values = values
    self._value_formatter = value_formatter or (lambda value: str(value))
    self._dialog = None
    self.set_click_callback(self._show_dialog)
    self.refresh()

  def _normalize_value(self, value: int) -> int:
    return min(self._values, key=lambda option: abs(option - value))

  def _get_current_value(self) -> int:
    try:
      value = int(self._params.get(self._param))
    except (TypeError, ValueError):
      value = self._values[0]
    return self._normalize_value(value)

  def refresh(self):
    self.set_value(self._value_formatter(self._get_current_value()))

  def _show_dialog(self):
    value_to_label = {value: self._value_formatter(value) for value in self._values}
    label_to_value = {label: value for value, label in value_to_label.items()}
    current_label = value_to_label[self._get_current_value()]

    def handle_selection(result: DialogResult):
      if result == DialogResult.CONFIRM and self._dialog is not None:
        self._params.put_nonblocking(self._param, label_to_value[self._dialog.selection])
        self.refresh()
      self._dialog = None

    self._dialog = MultiOptionDialog(self.text, list(label_to_value.keys()), current=current_label, callback=handle_selection)
    gui_app.push_widget(self._dialog)


class TogglesLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()

    self._personality_toggle = BigMultiParamToggle("driving personality", "LongitudinalPersonality", ["aggressive", "standard", "relaxed"])
    self._experimental_btn = BigParamControl("experimental mode", "ExperimentalMode")
    improved_stopped_approach = BigParamControl("improved stopped approach", "ImprovedStoppedApproach")
    self._ev_power_limiter_btn = BigParamControl("ev power limiter", "EvPowerLimiter")
    self._ev_power_limiter_debug_btn = BigParamControl("ev power limiter debug", "EvPowerLimiterDebug")
    self._ev_power_limit_btn = BigIntParamSelect("ev power limit", "EvPowerLimitKw", list(range(10, 68)),
                                                 value_formatter=lambda value: f"{value} kW")
    is_metric_toggle = BigParamControl("use metric units", "IsMetric")
    ldw_toggle = BigParamControl("lane departure warnings", "IsLdwEnabled")
    always_on_dm_toggle = BigParamControl("always-on driver monitor", "AlwaysOnDM")
    record_front = BigParamControl("record & upload driver camera", "RecordFront", toggle_callback=restart_needed_callback)
    record_mic = BigParamControl("record & upload mic audio", "RecordAudio", toggle_callback=restart_needed_callback)
    enable_openpilot = BigParamControl("enable sunnypilot", "OpenpilotEnabledToggle", toggle_callback=restart_needed_callback)
    self._ev_power_limiter_debug_btn.set_visible(lambda: ui_state.params.get_bool("EvPowerLimiter"))
    self._ev_power_limit_btn.set_visible(lambda: ui_state.params.get_bool("EvPowerLimiter"))

    self._scroller.add_widgets([
      self._personality_toggle,
      self._experimental_btn,
      improved_stopped_approach,
      self._ev_power_limiter_btn,
      self._ev_power_limiter_debug_btn,
      self._ev_power_limit_btn,
      is_metric_toggle,
      ldw_toggle,
      always_on_dm_toggle,
      record_front,
      record_mic,
      enable_openpilot,
    ])

    # Toggle lists
    self._refresh_toggles = (
      ("ExperimentalMode", self._experimental_btn),
      ("ImprovedStoppedApproach", improved_stopped_approach),
      ("EvPowerLimiter", self._ev_power_limiter_btn),
      ("EvPowerLimiterDebug", self._ev_power_limiter_debug_btn),
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
    self._ev_power_limit_btn.refresh()
