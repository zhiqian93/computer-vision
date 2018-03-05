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