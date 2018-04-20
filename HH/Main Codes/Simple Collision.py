from HH import Dr & Nurse Skeleton

class collision():
    def __init__(self):
        # Initialize Kinect
        self.kinect = PyKinectRuntime.PyKinectRuntime(
            PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Depth)
        print(type(self.kinect))