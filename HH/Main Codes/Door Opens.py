import cv2
from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
import numpy as np

class door_open():
    def __init__(self):
        # Initialize Kinect
        self.kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Depth)
        print(type(self.kinect))
        # Get one color picture frame
        self.frame = None
        while(True):
            if (self.kinect.has_new_depth_frame()):
                self.frame = self.kinect.get_last_depth_frame()
                break

        # Reshape in array of 424 x 512
        self.frame = self.frame.reshape((424, 512))
        self.data = np.asarray(self.frame)

    # Kinect is able to track from 0.5m to 4.5 m
    # pixel values: 0 - 4500 (near - far)
    # https://stackoverflow.com/questions/45435441/range-of-values-for-depth-images-generated-by-kinect-v1

    def model_find(self):
        # use models to find the door directly
        pass

    def background_find(self):
        # use background subtraction:
        # Assuming first frame is background only
        self.firstFrame = None

        while True:
            # if the first frame is None, initialize it
            if self.firstFrame is None:
                self.firstFrame = self.frame
                continue

            frameDelta = cv2.absdiff(self.firstFrame, self.frame)
            thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]

            # dilate the thresholded image to fill in holes, then find contours on thresholded image
            thresh = cv2.dilate(thresh, None, iterations=2)
            (_, contours, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_SIMPLE)

            # loop over the contours
            for c in contours:  # if the contour is too small, ignore it - need to add it position of door as well
                if cv2.contourArea(c) < 500:
                    continue

                else:
                    return True

    def manual_find(self):
        # use manual method to find where door is:
        # data[row, columns] chooses the spot, so as the door is in the middle right corner,
        # row: 78 - 345
        # column: 336 - 394
        door = self.data[78:345, 336:394]

        # Assume door is closed initially, opening the door will activate.
        print(np.average(door))

        # hard code version: (if average distance > 3m)
        if np.average(door) > 3000:
            return True




