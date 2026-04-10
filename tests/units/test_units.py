from unittest import TestCase

from frc_python.units.distance import inches, meters
from frc_python.units.mass import kilograms
from frc_python.units.time import minutes, seconds
from frc_python.units.velocity import meters_per_second


class TestUnit(TestCase):
    def test_muldim(self):
        m = kilograms(5)
        s = minutes(2)
        ms = m.muldim(s)
        self.assertEqual(ms.unit, "(kilograms * minutes)")
        self.assertEqual(ms.raw, 5 * 2 * 60)
        self.assertEqual(ms.in_current(), 5 * 2)

    def test_divdim(self):
        m = inches(10)
        s = seconds(3)
        mps = m.divdim(s)
        self.assertEqual(mps.unit, "(inches / seconds)")
        self.assertAlmostEqual(mps.in_current(), 10 / 3)

    def test_str(self):
        self.assertEqual(str(meters_per_second(5.5)), "5.5 (meters / seconds)")


class TestUnitPerUnit(TestCase):
    def test_mulr(self):
        val = meters_per_second(5)
        multiplied = val.mulr(seconds(2))
        self.assertEqual(multiplied.raw, 5 * 2)
