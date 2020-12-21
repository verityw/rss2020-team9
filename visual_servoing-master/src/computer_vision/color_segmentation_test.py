# -*- coding: utf-8 -*-

"""
Unit tests for color_segmentation.py
"""

import unittest
import cv2
import numpy as np
from color_segmentation import *


class TestLookahead(unittest.TestCase):
    """Unit tests aiming to test lookahead"""

    def test_simple_image_lookahead(self):
        black = [0,0,0]
        white = [255,255,255]
        # orig = np.array([*[[white for i in range(3)] for j in range(10)]])
        orig = np.array([[[255, 255, 255],
        [255, 255, 255],
        [255, 255, 255]],

       [[255, 255, 255],
        [255, 255, 255],
        [255, 255, 255]],

       [[255, 255, 255],
        [255, 255, 255],
        [255, 255, 255]],

       [[255, 255, 255],
        [255, 255, 255],
        [255, 255, 255]],

       [[255, 255, 255],
        [255, 255, 255],
        [255, 255, 255]],

       [[255, 255, 255],
        [255, 255, 255],
        [255, 255, 255]],

       [[255, 255, 255],
        [255, 255, 255],
        [255, 255, 255]],

       [[255, 255, 255],
        [255, 255, 255],
        [255, 255, 255]],

       [[255, 255, 255],
        [255, 255, 255],
        [255, 255, 255]],

       [[255, 255, 255],
        [255, 255, 255],
        [255, 255, 255]]])
        # expected = np.array([[black for _ in range(3)],[black for _ in range(3)],[black for _ in range(3)],[black for i in range(3)],[],[],[],[],[],[]])
        # expected = np.array([*[[black for i in range(3)] for j in range(5)],*[[white for i in range(3)] for j in range(3)],*[[black for i in range(3)] for j in range(2)]])
        expected = np.array([[[  0,   0,   0],
        [  0,   0,   0],
        [  0,   0,   0]],

       [[  0,   0,   0],
        [  0,   0,   0],
        [  0,   0,   0]],

       [[  0,   0,   0],
        [  0,   0,   0],
        [  0,   0,   0]],

       [[  0,   0,   0],
        [  0,   0,   0],
        [  0,   0,   0]],

       [[  0,   0,   0],
        [  0,   0,   0],
        [  0,   0,   0]],

       [[255, 255, 255],
        [255, 255, 255],
        [255, 255, 255]],

       [[255, 255, 255],
        [255, 255, 255],
        [255, 255, 255]],

       [[255, 255, 255],
        [255, 255, 255],
        [255, 255, 255]],

       [[  0,   0,   0],
        [  0,   0,   0],
        [  0,   0,   0]],

       [[  0,   0,   0],
        [  0,   0,   0],
        [  0,   0,   0]]])

        actual = lookahead(orig)
        np.testing.assert_array_equal(expected,actual)

if __name__ == '__main__':
    unittest.main()
