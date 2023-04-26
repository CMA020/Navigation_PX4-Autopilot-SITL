#!/usr/bin/env python3

import asyncio

import self as self
from mavsdk import System
from mavsdk.offboard import (PositionNedYaw, OffboardError, VelocityNedYaw,VelocityBodyYawspeed,Attitude)
import math
import threading
import pickle
import cv2
import gi
import numpy as np
gi.require_version('Gst', '1.0')
from gi.repository import Gst
global flag
global dist,angle_i
global roll,yaw,pitch
roll=0
pitch=0
import time
flag=0
dist =0

with open("calibration.pkl", "rb") as f:
    data = pickle.load(f)
    cMat = data[0]
    dcoeff = data[1]

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

class Video():
    """BlueRov video capture class constructor
    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
    """

    def __init__(self, port=5600):
        """Summary
        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self._frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]
        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array
        Args:
            sample (TYPE): Description
        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        """ Get Frame
        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """Check if frame is available
        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)

    def run(self):
        """ Get frame to update _frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame

        return Gst.FlowReturn.OK
def vid():
    global flag
    global dist
    global roll,yaw,pitch
    video = Video()
    print("dawjhdjah")
    while True:
        # Wait for the next frame
        if not video.frame_available():
            continue
        frame = video.frame()
        dt = cv2.aruco.DetectorParameters_create()
        dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_100)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, dict, parameters=dt)
        R_flip=np.zeros((3,3),dtype=np.float32)
        R_flip[0,0]=1.0
        R_flip[1,1]=-1.0
        R_flip[2,2]=-1.0
        if ids is not None:

            for id in ids:
                # print(id)
                # Set the length of the ID detected.
                if (id[0] == 0):
                    flag=1
                    aruco_len = 1
                    # Get the rotation vec and translation vec of the camera to the aruco I believe. can use this to control the quad.
                    rvecs, tvecs,_ = cv2.aruco.estimatePoseSingleMarkers(corners[0], aruco_len, cMat, dcoeff)
                    dist = np.linalg.norm(tvecs)
                    cv2.aruco.drawDetectedMarkers(frame,corners)

                    yaw = rvecs[0, 0, 2]


                    print(dist)
                elif (id[0] == 1):
                    flag=2
                    aruco_len =1
                    # Get the rotation vec and translation vec of the camera to the aruco I believe. can use this to control the quad.
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[0], aruco_len, cMat, dcoeff)
                    dist = np.linalg.norm(tvecs)
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    yaw = rvecs[0, 0, 2]

                    print(dist)
                elif (id[0] == 2):
                    flag = 3
                    aruco_len = 1
                    # Get the rotation vec and translation vec of the camera to the aruco I believe. can use this to control the quad.
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[0], aruco_len, cMat, dcoeff)
                    dist = np.linalg.norm(tvecs)
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    yaw = rvecs[0, 0, 2]
                    cv2.aruco.drawDetectedMarkers(frame, corners)

                    print(dist)
                else :
                    flag=0
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
class thread(threading.Thread):
    def __init__(self, thread_name, thread_ID):
        threading.Thread.__init__(self)
        self.thread_name = thread_name
        self.thread_ID = thread_ID

        # helper function to execute the threads


    def run(self):
        # print(str(self.thread_name) + " " + str(self.thread_ID));
        vid()
thread1 = thread("vid", 1000)
async def run():
    global flag,angle_i
    global dist
    global yaw,roll,pitch
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("-- Arming")
    await drone.action.arm()
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await drone.offboard.start()
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.1, -1.5, 170))
    # time.sleep(1)
    await asyncio.sleep(7)
    x=2.5
    y=2.2
    s=0.8

    while (flag!=4):
        if (flag == 0 ):
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(s, 0.0, 0.0, 0.0))
        elif (flag == 1 and dist >= x):
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(s, 0.0, 0.0, 0.0))
        elif (flag == 2 and dist >= x):
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(s, 0.0, 0.0, 0.0))
        elif (flag ==1 and dist<=x):
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(s, 0.0, 0.0, 0.0))
            async for attitude in drone.telemetry.attitude_euler():
                angle_i = attitude.yaw_deg
                break
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, angle_i+85))
            await asyncio.sleep(5)
        elif (flag ==2 and dist<=x):
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            async for attitude in drone.telemetry.attitude_euler():
                angle_i = attitude.yaw_deg
                break
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, angle_i-85))
            await asyncio.sleep(5)
        elif (flag == 3 and dist >= y):
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.8, 0.0, 0.0, 0.0))
        elif (flag == 3 and dist <= y):
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await drone.offboard.stop()
            await drone.action.land()
            time.sleep(2)
            flag=4
    print("--Mission Complete")
if __name__ == "__main__":
    # Run the asyncio loop
    thread1.start()
    asyncio.run(run())

