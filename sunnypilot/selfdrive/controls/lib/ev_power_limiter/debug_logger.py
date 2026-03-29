"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import logging
import os
import time
from logging.handlers import RotatingFileHandler

from openpilot.system.hardware import PC

LOG_DIR = os.path.join(os.path.expanduser("~"), ".comma", "log") if PC else "/data/log"
LOG_FILE = os.path.join(LOG_DIR, "ev_power_limiter.log")

# 3 files x 512KB = 1.5MB max total
MAX_BYTES = 512 * 1024
BACKUP_COUNT = 2


def get_logger() -> logging.Logger:
  logger = logging.getLogger("ev_power_limiter")
  if not logger.handlers:
    logger.setLevel(logging.DEBUG)
    os.makedirs(LOG_DIR, exist_ok=True)
    handler = RotatingFileHandler(LOG_FILE, maxBytes=MAX_BYTES, backupCount=BACKUP_COUNT)
    handler.setFormatter(logging.Formatter("%(asctime)s %(message)s", datefmt="%H:%M:%S"))
    logger.addHandler(handler)
  return logger


class DebugLogger:
  def __init__(self):
    self.enabled = False
    self.logger = get_logger()
    self.last_log_time = 0.0
    self.log_interval = 0.5  # log every 500ms to avoid flooding

  def log_power_limiter(self, v_ego, a_ego, pitch, estimated_power_kw,
                        max_accel, power_limit_kw, state, source):
    if not self.enabled:
      return
    now = time.monotonic()
    if now - self.last_log_time < self.log_interval:
      return
    self.last_log_time = now
    self.logger.debug(
      f"PWR v={v_ego:.1f}m/s a_ego={a_ego:.2f}m/s2 pitch={pitch:.4f}rad "
      f"est={estimated_power_kw:.1f}kW lim={power_limit_kw}kW "
      f"a_max={max_accel:.2f}m/s2 state={state} src={source}"
    )

  def log_planner_output(self, a_target, v_target_source, accel_clip_lo, accel_clip_hi,
                         should_stop, has_lead):
    if not self.enabled:
      return
    now = time.monotonic()
    if now - self.last_log_time < self.log_interval:
      return
    self.last_log_time = now
    self.logger.debug(
      f"CMD a_cmd={a_target:.2f}m/s2 clip=[{accel_clip_lo:.2f},{accel_clip_hi:.2f}] "
      f"src={v_target_source} stop={should_stop} lead={has_lead}"
    )

  def log_stopped_approach(self, v_ego, lead_dist, lead_speed, extra_stop,
                           approach_brake, personality):
    if not self.enabled:
      return
    now = time.monotonic()
    if now - self.last_log_time < self.log_interval:
      return
    self.last_log_time = now
    self.logger.debug(
      f"STOP v={v_ego:.1f}m/s lead_d={lead_dist:.1f}m lead_v={lead_speed:.1f}m/s "
      f"extra={extra_stop:.1f}m brake={approach_brake:.1f} pers={personality}"
    )

  def log_event(self, msg: str):
    if not self.enabled:
      return
    self.logger.info(f"EVENT {msg}")
