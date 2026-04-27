from cereal import log

from openpilot.common.params import UnknownKeyName
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl, BigMultiParamToggle
from openpilot.selfdrive.ui.sunnypilot.mici.widgets.cycling_int_button import CyclingIntButton
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.settings.common import restart_needed_callback
from openpilot.selfdrive.ui.ui_state import ui_state

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

    self._scroller.add_widgets([
      self._personality_toggle,
      self._experimental_btn,
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
      ("IsMetric", is_metric_toggle),
      ("IsLdwEnabled", ldw_toggle),
      ("AlwaysOnDM", always_on_dm_toggle),
      ("RecordFront", record_front),
      ("RecordAudio", record_mic),
      ("OpenpilotEnabledToggle", enable_openpilot),
    )

    # EV limiter + log upload — only rendered if params_pyx.so has been
    # rebuilt to know about our custom keys. On a stock prebuilt library
    # any probe raises UnknownKeyName, and any OTHER exception (stale
    # .so, corrupted widget, missing upstream dep) should ALSO not wipe
    # out the Toggles panel. Catch broadly and keep going — user can
    # still reach experimental/metric/record/etc.
    self._cycling_refresh = ()
    try:
      ui_state.params.get_bool("EVLimiterEnabled")
      ev_widgets_ok = True
    except Exception:
      ev_widgets_ok = False

    if ev_widgets_ok:
      try:
        log_upload = BigParamControl("upload CAN logs", "LogUploadEnabled")
        ev_limiter = BigParamControl("EV power limiter", "EVLimiterEnabled")
        ev_power_thr = CyclingIntButton(
          "EV limiter power threshold",
          "EVLimiterPowerThresholdKW",
          values=[20, 30, 40, 50, 60],
          suffix=" kW",
          default=40,
        )
        ev_dte_floor = CyclingIntButton(
          "EV limiter DTE floor",
          "EVLimiterDTEFloor",
          values=[1, 3, 5, 10, 20, 50],
          suffix="",
          default=5,
        )
        ev_max_gap = CyclingIntButton(
          "EV limiter max gap",
          "EVLimiterMaxGapMph",
          values=[3, 5, 7, 10, 15],
          suffix=" mph",
          default=5,
        )
        self._scroller.add_widgets([log_upload, ev_limiter, ev_power_thr, ev_dte_floor, ev_max_gap])
        self._refresh_toggles = self._refresh_toggles + (
          ("LogUploadEnabled", log_upload),
          ("EVLimiterEnabled", ev_limiter),
        )
        self._cycling_refresh = (ev_power_thr, ev_dte_floor, ev_max_gap)
      except Exception as e:  # widget constructors must not take down the panel
        print(f"[toggles] EV-limiter widget init failed: {type(e).__name__}: {e}")

    # Calibration box-check bypass (re-applied from older branch).
    # Off by default: the upstream PITCH/YAW box check runs normally.
    # Toggling on disables only the box check — the spread check still runs,
    # so genuine mount shifts will still trigger recalibrating.
    # Same defensive try/except pattern as the EV widgets above.
    try:
      ui_state.params.get_bool("CalibrationBoxCheckDisabled")
      cal_widget_ok = True
    except Exception:
      cal_widget_ok = False

    if cal_widget_ok:
      try:
        calib_bypass = BigParamControl("disable calibration box check",
                                       "CalibrationBoxCheckDisabled")
        self._scroller.add_widgets([calib_bypass])
        self._refresh_toggles = self._refresh_toggles + (
          ("CalibrationBoxCheckDisabled", calib_bypass),
        )
      except Exception as e:
        print(f"[toggles] calibration-bypass widget init failed: {type(e).__name__}: {e}")

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

    # Refresh toggles from params to mirror external changes. One bad key
    # must not stop the loop — if a param has gone stale (e.g. params_pyx.so
    # rebuilt without a key), keep going so the rest of the panel still
    # reflects live state.
    for key, item in self._refresh_toggles:
      try:
        item.set_checked(ui_state.params.get_bool(key))
      except Exception:
        pass

    # Refresh cycling-int displays (EV limiter tunables set via SSH etc.)
    for btn in getattr(self, "_cycling_refresh", ()):
      try:
        btn._refresh_display()
      except Exception:
        pass
