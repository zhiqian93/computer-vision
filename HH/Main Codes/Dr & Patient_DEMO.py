from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
import ctypes
import pygame
import sys
import numpy as np
import cv2
import time
import math

import importlib

#importlib.import_module('Background Subtraction', package='HH')

if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread

# colors for drawing different bodies
SKELETON_COLORS = [pygame.color.THECOLORS["red"],
                   pygame.color.THECOLORS["blue"],
                   pygame.color.THECOLORS["green"],
                   pygame.color.THECOLORS["orange"],
                   pygame.color.THECOLORS["purple"],
                   pygame.color.THECOLORS["yellow"],
                   pygame.color.THECOLORS["violet"]]

# Capture bodies
class BodyGameRuntime(object):
    def __init__(self):

        pygame.init()
        myfont = pygame.font.SysFont("monospace", 15)

        # Video length in seconds
        self._captureTime = 21600
        self.dr_id = 0
        self.patient_id = 0
        self.id = 0
        self.body_position = (0, 0)
        self.tracked_bodies = []

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, self._infoObject.current_h >> 1),
                                               pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE, 32)

        pygame.display.set_caption("CSC Hand Hygiene Kinect V19")

        # Loop until the user clicks the close button.
        self._done = False

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Kinect runtime object, we want only depth and body frames
        self._kinect = PyKinectRuntime.PyKinectRuntime(
            PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Body)

        # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
        self._frame_surface = pygame.Surface(
            (self._kinect.depth_frame_desc.Width, self._kinect.depth_frame_desc.Height), 0, 24)

        # here we will store skeleton data
        self._bodies = None

        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode(
            (self._kinect.depth_frame_desc.Width, self._kinect.depth_frame_desc.Height),
            pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE, 32)

    def draw_body_bone(self, joints, jointPoints, depth, joint0, joint1):
        joint0State = joints[joint0].TrackingState;
        joint1State = joints[joint1].TrackingState;

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked):
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return

        # ok, at least one is good
        start = (jointPoints[joint0].x, jointPoints[joint0].y)
        end = (jointPoints[joint1].x, jointPoints[joint1].y)

        try:
            pygame.draw.line(self._frame_surface, depth, start, end, 4)
        except:  # need to catch it due to possible invalid positions (with inf)
            pass

    def draw_body(self, joints, jointPoints, depth):
        # Torso
        self.draw_body_bone(joints, jointPoints, depth, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck);
        self.draw_body_bone(joints, jointPoints, depth, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder);
        self.draw_body_bone(joints, jointPoints, depth, PyKinectV2.JointType_SpineShoulder,
                            PyKinectV2.JointType_SpineMid);
        self.draw_body_bone(joints, jointPoints, depth, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase);
        self.draw_body_bone(joints, jointPoints, depth, PyKinectV2.JointType_SpineShoulder,
                            PyKinectV2.JointType_ShoulderRight);
        self.draw_body_bone(joints, jointPoints, depth, PyKinectV2.JointType_SpineShoulder,
                            PyKinectV2.JointType_ShoulderLeft);
        self.draw_body_bone(joints, jointPoints, depth, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight);
        self.draw_body_bone(joints, jointPoints, depth, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft);

        # Right Arm
        self.draw_body_bone(joints, jointPoints, depth, PyKinectV2.JointType_ShoulderRight,
                            PyKinectV2.JointType_ElbowRight);
        self.draw_body_bone(joints, jointPoints, depth, PyKinectV2.JointType_ElbowRight,
                            PyKinectV2.JointType_WristRight);
        self.draw_body_bone(joints, jointPoints, depth, PyKinectV2.JointType_WristRight,
                            PyKinectV2.JointType_HandRight);
        self.draw_body_bone(joints, jointPoints, depth, PyKinectV2.JointType_HandRight,
                            PyKinectV2.JointType_HandTipRight);
        self.draw_body_bone(joints, jointPoints, depth, PyKinectV2.JointType_WristRight,
                            PyKinectV2.JointType_ThumbRight);

        # Left Arm
        self.draw_body_bone(joints, jointPoints, depth, PyKinectV2.JointType_ShoulderLeft,
                            PyKinectV2.JointType_ElbowLeft);
        self.draw_body_bone(joints, jointPoints, depth, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft);
        self.draw_body_bone(joints, jointPoints, depth, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft);
        self.draw_body_bone(joints, jointPoints, depth, PyKinectV2.JointType_HandLeft,
                            PyKinectV2.JointType_HandTipLeft);
        self.draw_body_bone(joints, jointPoints, depth, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_ThumbLeft);

        self.body_position = (jointPoints[PyKinectV2.JointType_SpineMid].x, jointPoints[PyKinectV2.JointType_SpineMid].y)

    def draw_depth_frame(self, frame, target_surface):
        if frame is None:  # some usb hub do not provide the infrared image. it works with Kinect studio though
            return
        target_surface.lock()
        f8 = np.uint8(frame.clip(1, 4000) / 16.)
        frame8bit = np.dstack((f8, f8, f8))
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame8bit.ctypes.data, frame8bit.size)
        del address
        target_surface.unlock()

    def run(self):
        # -------- Main Program Loop -----------
        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'DIVX')
        out = cv2.VideoWriter('test.mkv', fourcc, 20.0, (self._frame_surface.get_width(), self._frame_surface.get_height()))
        start_time = time.time()
        previous_time = start_time
        delta_minutes = 0
        dr_track_id = 7
        patient_track_id = 7
        case = " "
        position_dr_righttip = None
        position_dr_lefttip = None
        position_dr_righthand = None
        position_dr_lefthand = None
        position_patient = None

        while not self._done:
            # --- Main event loop
            key = cv2.waitKey(1) & 0xFF
            for event in pygame.event.get():  # User did something
                if event.type == pygame.QUIT:  # If user clicked close
                    self._done = True  # Flag that we are done so we exit this loop

                elif event.type == pygame.VIDEORESIZE:  # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'],
                                                           pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE, 32)

            # --- Game logic should go here
            # --- Getting frames and drawing
            # --- Woohoo! We've got a color frame! Let's fill out back buffer surface with frame's data
            if self._kinect.has_new_depth_frame():
                frame = self._kinect.get_last_depth_frame()
                self.draw_depth_frame(frame, self._frame_surface)
                frame = None

            # --- Cool! We have a body frame, so can get skeletons
            if self._kinect.has_new_body_frame():
                # print("body frame detected")
                self._bodies = self._kinect.get_last_body_frame()

            # --- draw skeletons to _frame_surface
            if self._bodies is not None:
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]
                    if not body.is_tracked:
                        continue

                    joints = body.joints
                    joint_points = self._kinect.body_joints_to_depth_space(joints)

                    # print(i, "  ", dr_track_id, "  ", patient_track_id)
                    # print(body.tracking_id)

                    # Initialize, track dr
                    if body.tracking_id not in self.tracked_bodies and (joint_points[PyKinectV2.JointType_SpineMid].x > 350):
                        # print("{0}: Dr!".format(body.tracking_id))
                        self.tracked_bodies.append(body.tracking_id)
                        dr_track_id = i
                        print("in dr")

                    if body.tracking_id not in self.tracked_bodies and (0 < joint_points[PyKinectV2.JointType_SpineMid].x < 150):
                        # print("{0}: Patient!".format(body.tracking_id))
                        self.tracked_bodies.append(body.tracking_id)
                        patient_track_id = i
                        print("in patient")

                    if i == dr_track_id and i != patient_track_id:
                        # print("dr tracked")
                        body_dr = self._bodies.bodies[dr_track_id]
                        joints_dr = body_dr.joints
                        # convert joint coordinates to color space
                        joint_points_dr = self._kinect.body_joints_to_depth_space(joints_dr)
                        self.draw_body(joints_dr, joint_points_dr, SKELETON_COLORS[3])
                        label_dr = pygame.font.SysFont("bold", 40).render("Clinician", 1, (155, 155, 155))
                        try:
                            self._frame_surface.blit(label_dr, (joint_points_dr[PyKinectV2.JointType_Head].x - 25,
                                                            joint_points_dr[PyKinectV2.JointType_Head].y - 50))
                        except TypeError:
                            continue
                        position_dr_lefttip = [joint_points_dr[PyKinectV2.JointType_HandTipLeft].x,
                                            joint_points_dr[PyKinectV2.JointType_HandTipLeft].y]
                        position_dr_righttip = [joint_points_dr[PyKinectV2.JointType_HandTipRight].x,
                                            joint_points_dr[PyKinectV2.JointType_HandTipRight].y]

                        position_dr_lefthand = [joint_points_dr[PyKinectV2.JointType_HandTipLeft].x,
                                            joint_points_dr[PyKinectV2.JointType_HandTipLeft].y]
                        position_dr_righthand = [joint_points_dr[PyKinectV2.JointType_HandTipRight].x,
                                             joint_points_dr[PyKinectV2.JointType_HandTipRight].y]

                    if i == patient_track_id and i != dr_track_id:
                        # print("patients tracked")
                        body_patient = self._bodies.bodies[patient_track_id]
                        joints_patient = body_patient.joints
                        # convert joint coordinates to color space
                        joint_points_patient = self._kinect.body_joints_to_depth_space(joints_patient)
                        self.draw_body(joints_patient, joint_points_patient, SKELETON_COLORS[6])
                        label_ptnt = pygame.font.SysFont("bold", 40).render("Patient", 1, (255, 155, 155))
                        try:
                            self._frame_surface.blit(label_ptnt,
                                                     (joint_points_patient[PyKinectV2.JointType_Head].x - 25,
                                                      joint_points_patient[PyKinectV2.JointType_Head].y - 50))
                        except TypeError:
                            continue
                        position_patient = [joint_points_patient[PyKinectV2.JointType_SpineMid].x,
                                            joint_points_patient[PyKinectV2.JointType_SpineMid].y]
                    # # Use else if instead of if, so that we track drs first
                    # else:
                    #     if i == patient_track_id and i != dr_track_id:
                    #         # print("patients tracked")
                    #         body_patient = self._bodies.bodies[patient_track_id]
                    #         joints_patient = body_patient.joints
                    #         # convert joint coordinates to color space
                    #         joint_points_patient = self._kinect.body_joints_to_depth_space(joints_patient)
                    #         self.draw_body(joints_patient, joint_points_patient, SKELETON_COLORS[6])
                    #         label_ptnt = pygame.font.SysFont("bold", 40).render("Patient", 1, (255, 155, 155))
                    #         try:
                    #             self._frame_surface.blit(label_ptnt,
                    #                                      (joint_points_patient[PyKinectV2.JointType_Head].x - 25,
                    #                                       joint_points_patient[PyKinectV2.JointType_Head].y - 50))
                    #         except TypeError:
                    #             continue
                    #         position_patient = [joint_points_patient[PyKinectV2.JointType_SpineMid].x,
                    #                             joint_points_patient[PyKinectV2.JointType_SpineMid].y]

                    # Distance between hands and patient spine mid
                    if position_patient and position_dr_lefttip and position_dr_righttip is not None:
                        distance_left = math.hypot((position_dr_lefttip[0] - position_patient[0]),
                                                  position_dr_lefttip[1] - position_patient[1])
                        distance_right = math.hypot((position_dr_righttip[0] - position_patient[0]),
                                                   position_dr_righttip[1] - position_patient[1])

                        if distance_left < 50 or distance_right < 50:
                            case = "touched patient"
                            label_intersect = pygame.font.SysFont("bold", 40).render("Touching Patient!", 1,
                                                                                     (100, 155, 255))
                            self._frame_surface.blit(label_intersect, (0, 10))

                    # Distance between left and right hand
                    if position_dr_righthand and position_dr_lefthand is not None:
                        distance = math.hypot((position_dr_lefthand[0] - position_dr_righthand[0]),
                                                position_dr_lefthand[1] - position_dr_righthand[1])

                        if distance < 30:
                            case = "HH Performed"
                            label_hh = pygame.font.SysFont("bold", 40).render("HH Performed!", 1, (34, 139, 34))
                            self._frame_surface.blit(label_hh, (300, 10))
                            pygame.draw.circle(self._frame_surface, (34, 193, 34), (int(position_dr_lefthand[0]),
                                                                                    int(position_dr_lefthand[1])), 20)
                            pygame.draw.circle(self._frame_surface, (34, 193, 34), (int(position_dr_righthand[0]),
                                                                                    int(position_dr_righthand[1])), 20)

                    if case == "touched patient":
                        label_intersect = pygame.font.SysFont("bold", 40).render("Touching Patient!", 1,
                                                                                 (100, 155, 255))

            pygame.draw.rect(self._frame_surface, (155, 155, 155), (350, 0, 200, 500), 4)
            pygame.draw.rect(self._frame_surface, (255, 155, 155), (0, 0, 150, 500), 4)

            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size)
            h_to_w = float(self._frame_surface.get_height()) / float(self._frame_surface.get_width())
            target_height = int(h_to_w * self._screen.get_width())
            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height));

            # pygame.surface to numpy array
            imgdata = pygame.surfarray.array3d(surface_to_draw)
            # swap two axes
            imgdata = np.rot90(imgdata, -1)
            # flip left right
            imgdata = np.fliplr(imgdata)

            self._screen.blit(surface_to_draw, (0, 0))
            surface_to_draw = None

            # --- Go ahead and update the screen with what we've drawn.
            # pygame.display.update()
            # pygame.display.flip()

            # --- Save video using opencv
            out.write(imgdata)
            cv2.imshow("HH Detection Frames", imgdata)

            delta_seconds = (time.time() - previous_time)
            if delta_seconds > 60:
                delta_minutes += 1
                previous_time = time.time()
                print("%s minutes" % delta_minutes)
            # print("--- %s seconds ---" % (time.time() - start_time))

            if cv2.waitKey(1) & 0xFF == ord('q') or ((time.time() - start_time) > self._captureTime):
                break

            # --- Limit to 60 frames per second
            self._clock.tick(60)

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
        pygame.quit()

__main__ = "HH detection"
game = BodyGameRuntime();
game.run();

