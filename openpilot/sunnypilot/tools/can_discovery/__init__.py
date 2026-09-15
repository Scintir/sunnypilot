"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

CAN discovery tooling: inventory every message seen on every panda bus in a route and
rank candidate bit-fields by correlation with an estimated tractive power. Used to find
high-voltage battery voltage / current / power (limit) signals on EV platforms that have
no DBC coverage for them yet.
"""
