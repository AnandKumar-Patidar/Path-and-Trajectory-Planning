#!/usr/bin/env python3
import unittest
from Diff_Drive_Path_Planning.cubic_spline_planner import CubicSplinePlanner

class TestCubicSplinePlanner(unittest.TestCase):
    def setUp(self):
        #input data
        self.x = [0, 2, 5, 6, 9, 12, 15]
        self.y = [0, 4, 8, 9, 12, 14, 16]
        self.planner = CubicSplinePlanner(self.x, self.y)

    def test_get_distance(self):
        pass
    def test_get_cubic_spline(self):
        pass
    def test_cubic_spline_course(self):
        pass
    def test_calc_s(self):
        pass
    def test_calc_position(self):
        pass

if __name__ =='__main__':
    unittest.main()

