import cv2
from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
import numpy as np

def door_open():
    # Initialize Kinect
    kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Depth)
    print(type(kinect))
    # Get one color picture frame
    frame = None
    while(True):
        if (kinect.has_new_depth_frame()):
            frame = kinect.get_last_depth_frame()
            break

    # Reshape in array of 424 x 512
    frame = frame.reshape((424, 512))
    data = np.asarray(frame)

    # use manual method to find where door is:
    # data[row, columns] chooses the spot, so as the door is in the middle right corner,
    # row: 78 - 345
    # column: 336 - 394

    door = data[78:345, 336:394]

    # Kinect is able to track from 0.5m to 4.5 m
    # pixel values: 0 - 4500 (near - far)
    # https://stackoverflow.com/questions/45435441/range-of-values-for-depth-images-generated-by-kinect-v1

    # Assume door is closed initially, opening the door will activate.
    # hard code version: (if average distance > 3m)
    print(np.average(door))

    if np.average(door) > 3000:
        return True




