# import the necessary packages
from imutils.object_detection import non_max_suppression
from imutils import paths
from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
import numpy as np
import argparse
import imutils
import cv2
import time

class collision():
    def __init__(self):
        # Initialize Kinect
        self.kinect = PyKinectRuntime.PyKinectRuntime(
            PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Depth)
        print(type(self.kinect))

