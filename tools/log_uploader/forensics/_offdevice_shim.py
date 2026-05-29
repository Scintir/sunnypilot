"""Off-device import shim for running forensics on a dev box.

The openpilot LogReader pulls in `openpilot.system.hardware`, which imports a
pile of on-device-only packages (requests, urllib3, serial, smbus2, ...). None
are needed to *parse* rlog files. This installs a meta-path finder that returns
auto-stub modules for those top-level packages and ALL their submodules, so any
`requests.Session | None` annotation or `from urllib3.response import X` resolves
to a throwaway type.

Import this module BEFORE importing openpilot.tools.lib.logreader.
"""
from __future__ import annotations
import sys, types
import importlib.abc, importlib.machinery

# Top-level packages that only exist on-device / for network IO.
_STUB_ROOTS = {
    "requests", "urllib3", "serial", "smbus2", "spidev", "usb", "pyudev",
    "websocket", "Crypto", "Cryptodome",
}


class _AutoModule(types.ModuleType):
    __path__: list = []  # mark as package so submodules are importable

    def __getattr__(self, name: str):
        if name.startswith("__"):
            raise AttributeError(name)
        # Any attribute is a fresh throwaway type — valid in `X | None` unions,
        # subclassable, and callable enough for module-load-time evaluation.
        return type(name, (object,), {})


class _StubFinder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    def find_spec(self, fullname, path=None, target=None):
        root = fullname.split(".", 1)[0]
        if root in _STUB_ROOTS:
            return importlib.machinery.ModuleSpec(fullname, self)
        return None

    def create_module(self, spec):
        return _AutoModule(spec.name)

    def exec_module(self, module):
        pass


def install():
    if not any(isinstance(f, _StubFinder) for f in sys.meta_path):
        sys.meta_path.insert(0, _StubFinder())


install()
