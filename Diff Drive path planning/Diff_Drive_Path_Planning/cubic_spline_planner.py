#!/usr/bin/env python3

import numpy as np
from scipy import interpolate


class CubicSplinePlanner:
    """
    A class to generate a smooth cubic spline path that passes through a set of waypoints.

    Args:
        x (array-like): The x-coordinates of the waypoints.
        y (array-like): The y-coordinates of the waypoints.

    Attributes:
        x (numpy.ndarray): The x-coordinates of the waypoints.
        y (numpy.ndarray): The y-coordinates of the waypoints.
       
    Methods:
        get_cubic_splines: Fits a cubic spline to the waypoints.
        
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y
        

    
    def get_cubic_splines(self, x, y, dt=0.1):

        """
        Computes the cubic splines for a set of (s, x) coordinates.

        Args:
            s (List[float]): List of s-coordinates of the waypoints.
            x (List[float]): List of x-coordinates of the waypoints.
            k (int): degree of polynomial

        Returns:
            Tuple: A tuple containing the spline coefficients, knots, and degree.
        """

        splines = CubicSpline(x, y)
        x_spline = np.arrange(self.cumdist[0], self.cumdist[-1], dt)
        y_spline = splines(x_spline)

        return x_spline, y_spline


