import unittest

from opendbc.car.hyundai.hyundaican import create_lfahda_mfc
from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests import common
from opendbc.safety.tests import test_hyundai as hyundai_tests
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP


class TestHyundaiElantraPortSafety(common.SafetyTestBase):
  DBC = "hyundai_can_generated"
  SAFETY_MODEL = CarParams.SafetyModel.hyundai
  SAFETY_PARAM = HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.HYBRID_GAS

  def test_lfahda_format_and_bus(self):
    # The working port changes the LFA display message length, not the allowed
    # address or bus. Test the actual controller output against the C hooks.
    for longitudinal in (False, True):
      for lda_button in (False, True):
        safety_param = self.SAFETY_PARAM | (HyundaiSafetyFlags.LONG if longitudinal else 0)
        self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.HAS_LDA_BUTTON if lda_button else 0)
        self.safety.set_safety_hooks(self.SAFETY_MODEL, safety_param)
        self.safety.init_tests()
        for controls_allowed in (False, True):
          self.safety.set_controls_allowed(controls_allowed)
          with self.subTest(longitudinal=longitudinal, lda_button=lda_button, controls_allowed=controls_allowed):
            address, data, bus = create_lfahda_mfc(self.packer, controls_allowed, 2 if controls_allowed else 0)
            self.assertEqual(len(data), 8)
            self.assertTrue(self._tx(libsafety_py.make_CANPacket(address, bus, data)))
            for wrong_length in range(8):
              self.assertFalse(self._tx(common.make_msg(0, 0x485, wrong_length)))
            for wrong_bus in (1, 2, 3):
              self.assertFalse(self._tx(common.make_msg(wrong_bus, 0x485, 8)))
            self.safety.set_relay_malfunction(True)
            self.assertFalse(self._tx(libsafety_py.make_CANPacket(address, bus, data)))
            self.safety.set_relay_malfunction(False)


class ElantraHEVCameraSafetyMixin:
  """Run the existing actuation suites with this port's combined safety flags."""
  SAFETY_PARAM_SP = 0

  def setUp(self):
    self.packer = common.CANPackerSafety(self.DBC)
    self.safety = libsafety_py.libsafety
    # SP flags select RX checks during hook initialization, so set them first.
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(self.SAFETY_MODEL, self.SAFETY_PARAM)
    self.safety.init_tests()
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP)

  def _user_gas_msg(self, gas):
    return self.packer.make_can_msg_safety("E_EMS11", 0, {"CR_Vcu_AccPedDep_Pos": gas})

  def test_hybrid_accelerator_wrong_bus_ignored(self):
    self._rx(self._user_gas_msg(0))
    for bus in (1, 2, 3):
      self._rx(self.packer.make_can_msg_safety("E_EMS11", bus, {"CR_Vcu_AccPedDep_Pos": 1}))
      self.assertFalse(self.safety.get_gas_pressed_prev())
    self._rx(self._user_gas_msg(1))
    self.assertTrue(self.safety.get_gas_pressed_prev())

  def test_valid_rx_configuration_and_timeout(self):
    # Exercise every selected RX check at the actual HEV/camera-SCC combination.
    # Camera SCC without LONG requires BCM_PO_11 even when no LDA flag is set.
    for _ in range(10):
      for msg in (self._user_gas_msg(0), self._speed_msg(0), self._user_brake_msg(0),
                  self._torque_driver_msg(0), self._button_msg(0)):
        self.assertTrue(self._rx(msg))
      if not self.SAFETY_PARAM & HyundaiSafetyFlags.LONG:
        self.assertTrue(self._rx(self._pcm_status_msg(False)))
        self.assertTrue(self._rx(self._acc_state_msg(False)))
      if not self.SAFETY_PARAM & HyundaiSafetyFlags.LONG or self.SAFETY_PARAM_SP & HyundaiSafetyFlagsSP.HAS_LDA_BUTTON:
        self.assertTrue(self._rx(self.packer.make_can_msg_safety("BCM_PO_11", 0, {"LDA_BTN": 0})))
    self.safety.set_controls_allowed(True)
    self.safety.safety_tick()
    self.assertTrue(self.safety.safety_config_valid())
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.set_timer(2_000_000)
    self.safety.safety_tick()
    self.assertFalse(self.safety.safety_config_valid())
    self.assertFalse(self.safety.get_controls_allowed())

  def test_bad_wheel_and_brake_checksums_disengage(self):
    for make_msg, byte, mask in ((self._speed_msg, 5, 0x40), (self._user_brake_msg, 6, 0x01)):
      self.setUp()
      self.assertTrue(self._rx(make_msg(0)))
      bad = make_msg(0)
      bad[0].data[byte] ^= mask
      self.safety.set_controls_allowed(True)
      self.assertFalse(self._rx(bad))
      self.assertFalse(self.safety.get_controls_allowed())


class TestHyundaiSafetyCameraSCCElantraHEV(ElantraHEVCameraSafetyMixin, hyundai_tests.TestHyundaiSafetyCameraSCC):
  SAFETY_PARAM = HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.HYBRID_GAS
  TX_MSGS = [[0x340, 0], [0x4F1, 2], [0x485, 0]]


class TestHyundaiSafetyCameraSCCElantraHEVLDA(TestHyundaiSafetyCameraSCCElantraHEV):
  SAFETY_PARAM_SP = HyundaiSafetyFlagsSP.HAS_LDA_BUTTON


class TestHyundaiLongitudinalSafetyCameraSCCElantraHEV(ElantraHEVCameraSafetyMixin, hyundai_tests.TestHyundaiLongitudinalSafetyCameraSCC):
  SAFETY_PARAM = HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.HYBRID_GAS | HyundaiSafetyFlags.LONG

  def test_camera_scc_rejects_radar_diagnostic(self):
    # The camera remains active; no radar-disable tester-present is allowed.
    for enabled in (False, True):
      self.safety.set_controls_allowed(enabled)
      self.assertFalse(self._tx(common.make_msg(0, 0x7D0, dat=bytes.fromhex("023e800000000000"))))


class TestHyundaiLongitudinalSafetyCameraSCCElantraHEVLDA(TestHyundaiLongitudinalSafetyCameraSCCElantraHEV):
  SAFETY_PARAM_SP = HyundaiSafetyFlagsSP.HAS_LDA_BUTTON


if __name__ == "__main__":
  unittest.main()
