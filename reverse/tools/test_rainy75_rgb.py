"""Unit tests for the port selection in rainy75_rgb.py (no hardware needed)."""
import subprocess
import unittest
from unittest import mock

import rainy75_rgb as r

# Trimmed `ioreg -r -c IOUSBHostDevice -l -w0` shape: device, two CDC ACM
# interfaces with their callout devices, and an unrelated device.
IOREG = """\
+-o Rainy 75 Pro@01100000  <class IOUSBHostDevice>
    "USB Product Name" = "Rainy 75 Pro"
  +-o IOUSBHostInterface@0  <class IOUSBHostInterface>
      "bInterfaceNumber" = 0
    +-o AppleUSBACMControl  <class AppleUSBACMControl>
      +-o IOSerialBSDClient  <class IOSerialBSDClient>
          "IOCalloutDevice" = "/dev/cu.usbmodem01234567891"
  +-o IOUSBHostInterface@3  <class IOUSBHostInterface>
      "bInterfaceNumber" = 3
    +-o AppleUSBACMControl  <class AppleUSBACMControl>
      +-o IOSerialBSDClient  <class IOSerialBSDClient>
          "IOCalloutDevice" = "/dev/cu.usbmodem01234567893"
+-o Some Board@02100000  <class IOUSBHostDevice>
    "USB Product Name" = "Some Board"
  +-o IOUSBHostInterface@0  <class IOUSBHostInterface>
      "bInterfaceNumber" = 0
    +-o IOSerialBSDClient  <class IOSerialBSDClient>
        "IOCalloutDevice" = "/dev/cu.usbmodem9"
"""


class PickConsole(unittest.TestCase):
    def test_interface_0_wins_over_order(self):
        self.assertEqual(r.pick_console([("/dev/ttyACM1", 3), ("/dev/ttyACM2", 0)]),
                         "/dev/ttyACM2")

    def test_single_port_firmware_without_interface(self):
        self.assertEqual(r.pick_console([("/dev/ttyACM0", None)]), "/dev/ttyACM0")

    def test_only_studio_port_still_returned(self):
        self.assertEqual(r.pick_console([("/dev/ttyACM1", 3)]), "/dev/ttyACM1")

    def test_none(self):
        self.assertIsNone(r.pick_console([]))


class IoregPorts(unittest.TestCase):
    def test_interfaces_and_only_rainy(self):
        done = subprocess.CompletedProcess([], 0, stdout=IOREG, stderr="")
        with mock.patch.object(r.subprocess, "run", return_value=done):
            self.assertEqual(r.ioreg_rainy_ports(),
                             [("/dev/cu.usbmodem01234567891", 0),
                              ("/dev/cu.usbmodem01234567893", 3)])


if __name__ == "__main__":
    unittest.main()
