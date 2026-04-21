#!/usr/bin/env python3
"""Scintir CAN log rsync uploader.

When offroad and `ScintirRsyncEnabled` is set, this daemon copies completed
route directories from the device to a user-managed server via rsync over
SSH. Routes are marked with a `user.scintir.uploaded` xattr so they are not
re-uploaded on subsequent passes. The daemon only runs offroad, so it never
competes with onroad processing.

Params used:
  ScintirRsyncEnabled      bool  master switch
  ScintirRsyncDestination  str   rsync destination, e.g. "user@host:/data/"
  ScintirRsyncWifiOnly     bool  skip if not on WiFi (default on)

SSH key path (user provisions once, device-local):
  /data/scintir/id_ed25519
"""

import os
import subprocess
import threading
import time

from cereal import log
import cereal.messaging as messaging
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import set_core_affinity
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware.hw import Paths
from openpilot.system.loggerd.xattr_cache import getxattr, setxattr


NetworkType = log.DeviceState.NetworkType

UPLOAD_ATTR_NAME = "user.scintir.uploaded"
UPLOAD_ATTR_VALUE = b"1"

SSH_KEY_PATH = "/data/scintir/id_ed25519"

# Camera/audio blobs are large and not useful for off-device CAN analysis.
RSYNC_EXCLUDES = (
  "*.hevc",
  "*.ts",
  "dcamera.*",
  "ecamera.*",
  "fcamera.*",
  "qcamera.*",
)

IDLE_POLL_S = 30
PER_ROUTE_TIMEOUT_S = 30 * 60
FAILURE_BACKOFF_THRESHOLD = 5  # after N consecutive failures, sleep longer
FAILURE_BACKOFF_MULT = 10       # idle poll multiplier applied after threshold


def _ssh_command() -> str:
  return (
    f"ssh -i {SSH_KEY_PATH} "
    "-o StrictHostKeyChecking=accept-new "
    "-o BatchMode=yes "
    "-o ConnectTimeout=10"
  )


def _route_is_complete(path: str) -> bool:
  try:
    return not any(f.endswith(".lock") for f in os.listdir(path))
  except OSError:
    return False


def _already_uploaded(path: str) -> bool:
  try:
    return getxattr(path, UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE
  except OSError:
    return False


def _mark_uploaded(path: str) -> None:
  try:
    setxattr(path, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE)
  except OSError:
    cloudlog.exception("scintir_rsync: failed to set uploaded xattr")


def _iter_pending_routes(root: str):
  try:
    entries = os.listdir(root)
  except OSError:
    return
  for name in sorted(entries):
    path = os.path.join(root, name)
    if not os.path.isdir(path):
      continue
    if not _route_is_complete(path):
      continue
    if _already_uploaded(path):
      continue
    yield path


def _rsync_route(source_dir: str, destination: str) -> bool:
  exclude_args: list[str] = []
  for pat in RSYNC_EXCLUDES:
    exclude_args += ["--exclude", pat]

  cmd = [
    "rsync",
    "-az",
    "--partial",
    "--timeout=60",
    "-e", _ssh_command(),
    *exclude_args,
    source_dir,
    destination,
  ]

  try:
    result = subprocess.run(
      cmd, capture_output=True, text=True, timeout=PER_ROUTE_TIMEOUT_S
    )
  except subprocess.TimeoutExpired:
    cloudlog.error("scintir_rsync: timeout rsyncing %s", source_dir)
    return False
  except Exception:
    cloudlog.exception("scintir_rsync: rsync invocation failed")
    return False

  if result.returncode != 0:
    cloudlog.error(
      "scintir_rsync: rsync failed rc=%d route=%s stderr=%s",
      result.returncode, source_dir, result.stderr[-500:],
    )
    return False

  cloudlog.info("scintir_rsync: uploaded %s", source_dir)
  return True


def _should_run(params: Params, sm: messaging.SubMaster) -> tuple[bool, str]:
  if not params.get_bool("IsOffroad"):
    return False, "onroad"
  # Scintir keys aren't in the prebuilt params_pyx.so allowlist on release
  # branches; a read of any Scintir* param raises UnknownKeyName. Trap that
  # so the daemon stays dormant rather than crashing.
  try:
    if not params.get_bool("ScintirRsyncEnabled"):
      return False, "disabled"
    if not params.get("ScintirRsyncDestination"):
      return False, "no destination"
    wifi_only = params.get_bool("ScintirRsyncWifiOnly")
  except UnknownKeyName:
    return False, "scintir params not yet registered (rebuild params_pyx.so to enable)"
  if not os.path.exists(SSH_KEY_PATH):
    return False, "missing ssh key"
  if wifi_only and sm["deviceState"].networkType != NetworkType.wifi:
    return False, "not on wifi"
  return True, ""


def main(exit_event: threading.Event | None = None) -> None:
  if exit_event is None:
    exit_event = threading.Event()

  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    cloudlog.exception("scintir_rsync: set_core_affinity failed")

  params = Params()
  sm = messaging.SubMaster(["deviceState"])
  root = Paths.log_root()
  consecutive_failures = 0

  while not exit_event.is_set():
    sm.update(0)

    ok, reason = _should_run(params, sm)
    if not ok:
      cloudlog.debug("scintir_rsync: idle (%s)", reason)
      time.sleep(IDLE_POLL_S)
      continue

    destination = params.get("ScintirRsyncDestination")
    for route_path in _iter_pending_routes(root):
      if exit_event.is_set():
        break
      if _rsync_route(route_path, destination):
        _mark_uploaded(route_path)
        consecutive_failures = 0
      else:
        consecutive_failures += 1
      ok, _ = _should_run(params, sm)
      if not ok:
        break

    # After repeated failures (bad key/host/destination) back off hard so we
    # don't thrash the logs and wake-ups. The counter only resets on a
    # successful rsync above -- that way a permanently misconfigured setup
    # stays in long-sleep mode instead of cycling through short-sleep bursts.
    if consecutive_failures >= FAILURE_BACKOFF_THRESHOLD:
      cloudlog.warning("scintir_rsync: %d consecutive failures, backing off", consecutive_failures)
      time.sleep(IDLE_POLL_S * FAILURE_BACKOFF_MULT)
    else:
      time.sleep(IDLE_POLL_S)


if __name__ == "__main__":
  main()
