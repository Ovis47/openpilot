"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import platform

from cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import SmartCruiseControlMap

MapState = VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.MapState


class TestSmartCruiseControlMap:

  def setup_method(self):
    self.params = Params()
    self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    self.reset_params()
    self.scc_m = SmartCruiseControlMap()

  def reset_params(self):
    self.params.put_bool("SmartCruiseControlMap", True, block=True)

    # TODO-SP: mock data from gpsLocation
    self.params.put("LastGPSPosition", "{}", block=True)
    self.params.put("MapTargetVelocities", "{}", block=True)

  def test_initial_state(self):
    assert self.scc_m.state == VisionState.disabled
    assert not self.scc_m.is_active
    assert self.scc_m.output_v_target == V_CRUISE_UNSET
    assert self.scc_m.output_a_target == 0.

  def test_system_disabled(self):
    self.params.put_bool("SmartCruiseControlMap", False, block=True)
    self.scc_m.enabled = self.params.get_bool("SmartCruiseControlMap")

    for _ in range(int(10. / DT_MDL)):
      self.scc_m.update(True, False, 0., 0., 0.)
    assert self.scc_m.state == VisionState.disabled
    assert not self.scc_m.is_active

  def test_disabled(self):
    for _ in range(int(10. / DT_MDL)):
      self.scc_m.update(False, False, 0., 0., 0.)
    assert self.scc_m.state == VisionState.disabled

  def test_transition_disabled_to_enabled(self):
    for _ in range(int(10. / DT_MDL)):
      self.scc_m.update(True, False, 0., 0., 0.)
    assert self.scc_m.state == VisionState.enabled

  # TODO-SP: mock data from modelV2 to test other states

  def _put_curve_at_position(self, valid: bool):
    # 現在地のすぐ先に 10 m/s の目標速度の点を置く
    position = {"latitude": 35.0, "longitude": 135.0, "bearing": 0.0, "valid": valid}
    self.mem_params.put("LastGPSPosition", json.dumps(position), block=True)
    self.mem_params.put("MapTargetVelocities", json.dumps([{"latitude": 35.0, "longitude": 135.0, "velocity": 10.0}]), block=True)

  def test_turning_with_valid_position(self):
    self._put_curve_at_position(valid=True)
    for _ in range(int(2. / DT_MDL)):
      self.scc_m.update(True, False, 20., 0., 25.)
    assert self.scc_m.state == MapState.turning

  def test_no_turning_before_gps_fix(self):
    self._put_curve_at_position(valid=False)
    for _ in range(int(2. / DT_MDL)):
      self.scc_m.update(True, False, 20., 0., 25.)
    assert self.scc_m.state == MapState.enabled
    assert self.scc_m.output_v_target == V_CRUISE_UNSET
