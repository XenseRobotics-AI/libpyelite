"""Offline surface checks for the LeRobot mainline adapter."""
import unittest

import libpyelite


class SDKCompatibilityTests(unittest.TestCase):
    def test_dh_package(self):
        package = libpyelite.KinematicsInfo()
        for name in ("dh_alpha_", "dh_a_", "dh_d_"):
            self.assertEqual(len(getattr(package, name)), 6)

    def test_driver_methods_are_bound(self):
        # Do not construct NativeDriver: its constructor creates SDK sockets.
        for name in ("setPayload", "getPrimaryPackage"):
            self.assertTrue(hasattr(libpyelite._native.NativeDriver, name))
