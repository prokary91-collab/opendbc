import unittest

from opendbc.car.hyundai.hyundaican import create_lfahda_mfc
from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests import common
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


if __name__ == "__main__":
  unittest.main()
