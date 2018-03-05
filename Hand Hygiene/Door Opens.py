# from pykinect2 import PyKinectRuntime
# from pykinect2 import PyKinectV2
# import pandas as pd
# import pygame
#
# kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth)
# test = PyKinectRuntime.PyKinectRuntime.get_last_depth_frame(kinect)
#
# test = pd.DataFrame(test)
#
# print(test)
# # # pygame.display.update()
# # # pygame.display.flip()
#
# from pykinect2 import PyKinectRuntime
# from pykinect2 import PyKinectV2
# import numpy as np
#
# kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth)
# depthframe = PyKinectRuntime.PyKinectRuntime.get_last_depth_frame(kinect)
#
# ptr_depth = np.ctypeslib.as_ctypes(depthframe.flatten())
# L = depthframe.size
#
# S = 1080*1920
# TYPE_CameraSpacePointArray = PyKinectV2._CameraSpacePoint * S
# csps1 = TYPE_CameraSpacePointArray()
# error_state = PyKinectRuntime.PyKinectRuntime._kinect._mapper.MapColorFrameToCameraSpace(L, ptr_depth, S, csps1)

import numpy as np
data = np.load('StableOutput_Depth4.npy')
print(data)