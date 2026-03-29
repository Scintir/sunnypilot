"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
from cereal import log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  LongitudinalMpc,
  get_stopped_equivalence_factor,
)
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.ev_power_limiter.debug_logger import DebugLogger

# Velocity threshold below which a lead is considered "stopped"
STOPPED_LEAD_V_THRESHOLD = 0.5  # m/s

# Extra distance subtracted from obstacle position for stopped leads (personality-dependent).
# This makes the MPC think the obstacle is closer, so the ego vehicle stops further away.
EXTRA_STOP_DISTANCE = {
  log.LongitudinalPersonality.relaxed: 4.0,
  log.LongitudinalPersonality.standard: 3.0,
  log.LongitudinalPersonality.aggressive: 1.0,
}

# Reduced comfort brake for stopped lead approach (personality-dependent).
# Lower value = system assumes it brakes less effectively = starts braking earlier.
APPROACH_COMFORT_BRAKE = {
  log.LongitudinalPersonality.relaxed: 1.8,
  log.LongitudinalPersonality.standard: 2.0,
  log.LongitudinalPersonality.aggressive: 2.3,
}


def get_stopped_equivalence_factor_sp(v_lead, comfort_brake):
  """Same as upstream but with parameterized comfort brake."""
  return (v_lead ** 2) / (2 * comfort_brake)


class LongitudinalMpcSP(LongitudinalMpc):
  def __init__(self, dt=DT_MDL):
    super().__init__(dt=dt)
    self.params_reader = Params()
    self.debug = DebugLogger()
    self.sp_frame = 0
    self.improved_stopped_approach = self.params_reader.get_bool("ImprovedStoppedApproach")

  def _update_params(self):
    if self.sp_frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.improved_stopped_approach = self.params_reader.get_bool("ImprovedStoppedApproach")
      self.debug.enabled = self.params_reader.get_bool("EvPowerLimiterDebug")

  def _compute_lead_obstacle(self, lead_xv, lead_status, v_lead, personality):
    """Compute obstacle distance for a lead vehicle.

    When improved_stopped_approach is enabled and the lead is stopped, uses a softer
    comfort brake (earlier braking) and adds an extra stop buffer.
    Otherwise falls back to the upstream equivalence factor.
    """
    if self.improved_stopped_approach and lead_status and v_lead < STOPPED_LEAD_V_THRESHOLD:
      extra_stop = EXTRA_STOP_DISTANCE.get(personality, 3.0)
      approach_brake = APPROACH_COMFORT_BRAKE.get(personality, 2.0)
      obstacle = lead_xv[:, 0] + get_stopped_equivalence_factor_sp(lead_xv[:, 1], approach_brake) - extra_stop
      self.debug.log_stopped_approach(self.x0[1], lead_xv[0, 0], v_lead,
                                      extra_stop, approach_brake, personality)
      return obstacle
    return lead_xv[:, 0] + get_stopped_equivalence_factor(lead_xv[:, 1])

  def update(self, radarstate, v_cruise, personality=log.LongitudinalPersonality.standard):
    self.sp_frame += 1
    self._update_params()

    if not self.improved_stopped_approach:
      super().update(radarstate, v_cruise, personality)
      return

    # Override lead obstacle computation, then delegate to parent for the rest.
    # We need to replicate the parent's lead processing since it's done inline,
    # but the solver setup (cruise obstacle, params, run, FCW) is identical.
    lead_xv_0 = self.process_lead(radarstate.leadOne)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)

    # Stash modified obstacles so _update_with_obstacles can use them
    lead_0_obstacle = self._compute_lead_obstacle(lead_xv_0, radarstate.leadOne.status,
                                                  radarstate.leadOne.vLead, personality)
    lead_1_obstacle = self._compute_lead_obstacle(lead_xv_1, radarstate.leadTwo.status,
                                                  radarstate.leadTwo.vLead, personality)

    self._run_mpc(radarstate, v_cruise, personality, lead_xv_0, lead_0_obstacle, lead_1_obstacle)

  def _run_mpc(self, radarstate, v_cruise, personality, lead_xv_0, lead_0_obstacle, lead_1_obstacle):
    """Set up and solve the MPC with pre-computed lead obstacles.

    The solver setup below mirrors LongitudinalMpc.update() — if the parent's
    update method changes, this must be kept in sync.
    """
    from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
    from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
      get_safe_obstacle_distance, get_T_FOLLOW,
      T_IDXS, T_DIFFS, N, COST_E_DIM, LEAD_DANGER_FACTOR,
      CRUISE_MIN_ACCEL, CRUISE_MAX_ACCEL,
      MPC_SOURCES, FCW_IDXS, CRASH_DISTANCE,
    )

    t_follow = get_T_FOLLOW(personality)
    v_ego = self.x0[1]
    self.status = radarstate.leadOne.status or radarstate.leadTwo.status

    # Cruise obstacle — identical to parent
    v_lower = v_ego + (T_IDXS * CRUISE_MIN_ACCEL * 1.05)
    v_upper = v_ego + (T_IDXS * CRUISE_MAX_ACCEL * 1.05)
    v_cruise_clipped = np.clip(v_cruise * np.ones(N + 1), v_lower, v_upper)
    cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, t_follow)

    x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])
    self.source = MPC_SOURCES[np.argmin(x_obstacles[0])]

    self.yref[:, :] = 0.0
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params[:, 0] = ACCEL_MIN
    self.params[:, 1] = ACCEL_MAX
    self.params[:, 2] = np.min(x_obstacles, axis=1)
    self.params[:, 3] = np.copy(self.a_prev)
    self.params[:, 4] = t_follow
    self.params[:, 5] = LEAD_DANGER_FACTOR

    self.run()
    if (np.any(lead_xv_0[FCW_IDXS, 0] - self.x_sol[FCW_IDXS, 0] < CRASH_DISTANCE) and
            radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0
