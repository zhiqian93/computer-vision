# computer-vision test

15 Dec 1st Commit: </br>

    Using the examples of PyKinectBodyGame.py, we constructed:
        - the skeleton on depth images
        - detection of hand hygiene once wrist is close to the other wrist
    To do:
        - Further refine HH movements
        - Differentiate clinicians, patients and nurses
        - Record data
        - Purchase/Research Potential Dev Boards
        - Or perhaps buy an Intel Real Sense Kit
        
16 Dec: Debug and Installing Library
    
    1. Fix issues of using Python 3.7 instead of Python 3.6
        - pygame is only available in 3.6
        - reset pip if have duplicated copies (verified later that pip is not the issue)
        - pip is disabled to install wheel files [Unresolved]
        
    2. Github files are located in D;\GitHub\
    
17 Dec:
    
    1. Files failing to compile due to attributes not found in pykinect2 library
        - Verified that the pykinect2 library was an outdated version
    
    2. Eye Clinic HH.py successfully executed 
    
    3. GitHub error, cant push/commit from Pycharm, have to be done in GitHub Desktop
    
26 Dec:
    
    Base python files:
        - Depth Skeleton [DONE]
        - Tracking Patient
            a. Track Chair and People in the chair
            b. Once detected Initiate "wait for HH"
        - Tracking HH
            a. use ESP8266 for handrub dispenser
            b. disp plates ready
            c. ESP8266 [haven't buy]
            d. Disp Box [haven't collect/print]
            
5 March: 

    Trying to use:
    kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth)
    test = PyKinectRuntime.PyKinectRuntime.get_last_depth_frame(kinect) 
    
    to get depth data but unsure, search more of it tomorrow
    
    test depth.py able to export depth data into numpy array
    
12 March:

    Door Open done
    Track HH needs to define rectangle shape of HH disp
    Dr and Nurse Skeleton need to differentiate from initial location
    Collision of bounding box haven't started
   
Important: Some camera are connected to Kinect but some are used with laptop webcam.
Regardless, parse camera video file as --v input.

Code Logic:
![picture](https://github.com/zhiqian93/computer-vision/blob/master/HH%20Structure.png)

13 March:

    Ignore bounding box, collision can be done in a simpler way:
    When the hand joints of the clinician is moving towards the patient, trigger HH moment 1.

23 April:
    
    cv.writer unable to write more than 2GB AVI files, most likely due to file type error.
    Solution could be:
    1. Use ffmpeg as an external codec
    2. Internally switch to DIVX (previously XVID) and output .mkv files.
    
    Also, automatically stops capturing once time is reached.
    
    To fix: Patient and Drs surface.blit clashes. [DONE]
    Find what is self.joints in PyKinectRuntime.py class KinectBody 
    
IMPORTANT:

     How to capture skeleton coordinates:
     In function draw_body(self, joints, jointPoints, depth),
     use jointPoints[PyKinectV2.JointType_Head].x
     
     Similarly in function run(self), use joint_points[PyKinectV2.JointType_Head].x 
     body = self._bodies reads from PyKinectRuntime.py. Mainly from KinectBodyFrameData 
     body = self._bodies.bodies[i] reads from class KinectBody, so should focus more on this specific class
     
     Find a way to read ctypes.cast in python
     
25 April:

    1. Initialize a dummy joint for us to check if its a dr/patient.
    2. Used Try and Error exception when blit is too close to screen sides.
    3. Solved patient turning into dr (vice versa) by:
    - append body.tracking_id to self.tracked_bodies
    - i = dr_id != patient_id
    4. Error in playing audio.
    5. Next step would be to link esp8266 to laptop/UP Board
    
    **Have to clean up the code for id, better just use one list to contain all detail, MUST DO LATER
    
2 May:

    1. UP Board WiFi hotspot inconsistent, use a separate nodeMCU and serial link together might be better
    2. For Python3, uninstall serial and pyserial, then install pyserial again. 
    3. Arduino encoding format is ASCII, but utf-8 works anyways.
    
4 May:
    
    1. Unable to track patients if door opens suddenly within a short timing.