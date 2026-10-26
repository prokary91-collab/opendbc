import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.fw_versions import match_fw_to_car_exact
from opendbc.car.hyundai.hyundaican import create_lfahda_mfc
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags, HyundaiSafetyFlags
from opendbc.car.structs import CarParams


class TestElantra2024Port(unittest.TestCase):
  def test_known_hybrid_firmware_matches_refresh_platform(self):
    # Firmware retained from the working 2024-25 HEV port. Exercise the matcher,
    # including the shared radar and both known EPS part numbers.
    for eps in (b'\xf1\x00CN7 MDPS C 1.00 1.00 56300BY670\x00 4CSHC100',
                b'\xf1\x00CN7 MDPS C 1.00 1.00 56300BY680\x00 4CSHC100'):
      for camera in (b'\xf1\x00CN7HMFC  AT AUS RHD 1.00 1.02 99210-AA500 230420',
                     b'\xf1\x00CN7HMFC  AT CAN LHD 1.00 1.05 99210-AA510 240509',
                     b'\xf1\x00CN7HMFC  AT USA LHD 1.00 1.03 99210-AA500 230918'):
        live_fw = {
          (0x7d0, None): {b'\xf1\x00CN7_ RDR -----      1.00 1.01 99110-AA500         '},
          (0x7d4, None): {eps},
          (0x7c4, None): {camera},
        }
        with self.subTest(eps=eps, camera=camera):
          self.assertEqual(match_fw_to_car_exact(live_fw, match_brand="hyundai", log=False), {CAR.HYUNDAI_ELANTRA_HEV_2024})
          live_fw[(0x7c4, None)] = {b'unknown camera firmware'}
          self.assertNotIn(CAR.HYUNDAI_ELANTRA_HEV_2024, match_fw_to_car_exact(live_fw, match_brand="hyundai", log=False))

  def test_hybrid_safety_configuration(self):
    for alpha_long in (False, True):
      with self.subTest(alpha_long=alpha_long):
        cp = CarInterface.get_params(CAR.HYUNDAI_ELANTRA_HEV_2024, gen_empty_fingerprint(), [], alpha_long, False, False)
        self.assertFalse(cp.dashcamOnly)
        self.assertFalse(cp.flags & HyundaiFlags.CANFD)
        self.assertTrue(cp.flags & HyundaiFlags.HYBRID)
        self.assertTrue(cp.flags & HyundaiFlags.CAMERA_SCC)
        self.assertEqual(cp.safetyConfigs[-1].safetyModel, CarParams.SafetyModel.hyundai)
        expected = HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.HYBRID_GAS
        if alpha_long:
          expected |= HyundaiSafetyFlags.LONG
        self.assertEqual(cp.safetyConfigs[-1].safetyParam, expected)
        self.assertEqual(cp.openpilotLongitudinalControl, alpha_long)
        self.assertEqual(cp.pcmCruise, not alpha_long)

  def test_lfahda_output_retains_eight_byte_format(self):
    packer = CANPacker(DBC[CAR.HYUNDAI_ELANTRA_HEV_2024][Bus.pt])
    for icon in (0, 1, 2, 3):
      with self.subTest(icon=icon):
        address, data, bus = create_lfahda_mfc(packer, True, icon)
        self.assertEqual((address, bus, len(data)), (0x485, 0, 8))
        self.assertEqual(data[4:], bytes(4))


if __name__ == "__main__":
  unittest.main()
