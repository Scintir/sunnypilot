import ctypes
import os
import time
from unittest import mock

from openpilot.common.test import OpenpilotTestCase
from openpilot.system.hardware import hardwared


class TestStallWatchdog(OpenpilotTestCase):
  def setUp(self):
    self.events = []
    self.patch = mock.patch.object(hardwared.cloudlog, "event", lambda name, **kw: self.events.append((name, kw)))
    self.patch.start()
    self.wd = hardwared.StallWatchdog()

  def tearDown(self):
    self.patch.stop()
    hardwared.faulthandler.cancel_dump_traceback_later()
    os.close(self.wd.fd)

  def test_normal_tick_is_silent(self):
    self.wd.tick(False)
    self.wd.mark("a")
    time.sleep(0.02)
    self.wd.mark("b")
    self.wd.tick(False)
    assert self.events == []
    assert os.path.getsize(hardwared.STALL_DUMP_PATH) == 0

  def test_gil_held_stall_is_reported_with_stacks(self):
    # PyDLL keeps the GIL during the foreign call, the worst case the watchdog must survive
    libc = ctypes.PyDLL(None)
    self.wd.tick(True)
    self.wd.mark("pre")
    libc.usleep(int((hardwared.STALL_DUMP_S + 0.5) * 1e6))
    self.wd.mark("stalled")
    self.wd.tick(True)

    assert len(self.events) == 1
    name, kw = self.events[0]
    assert name == "hardwared slow tick"
    assert kw["started"] is True
    assert kw["period"] > hardwared.SLOW_TICK_WARN_S
    assert kw["slowest_phases"][0][0] == "stalled"
    assert "Thread" in kw["stacks"] and __file__ in kw["stacks"]

    # dump was drained: the next normal tick is quiet again
    self.wd.tick(True)
    assert len(self.events) == 1
    assert os.path.getsize(hardwared.STALL_DUMP_PATH) == 0
