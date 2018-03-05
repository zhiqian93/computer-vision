from pykinect2 import PyKinectRuntime
from pykinect2 import PyKinectV2
import pandas as pd
import pygame

kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth)
test = PyKinectRuntime.PyKinectRuntime.get_last_depth_frame(kinect)

test = pd.DataFrame(test)

print(test)
# # pygame.display.update()
# # pygame.display.flip()