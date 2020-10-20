"""
Example script to detect aruco markers from an image using OpenCV
"""
import json
import numpy as np
import cv2
from cv2 import aruco
from utils.aruco_utils import aruco_poses_to_transforms
from utils.select_video_source import select_video_source


# Select the camera source by setting this
VIDEO_SOURCE = "ffmpeg"  # Options: 'gstreamer', 'webcam' or 'opencv'

ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_50)
ARUCO_DETECTER_PARAMETERS = aruco.DetectorParameters_create()
# Let's set some aruco detection parameters to make the marker
# detection a bit more stable
ARUCO_DETECTER_PARAMETERS.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
ARUCO_DETECTER_PARAMETERS.cornerRefinementWinSize = 5
ARUCO_DETECTER_PARAMETERS.minMarkerDistanceRate = 0.05
ARUCO_DETECTER_PARAMETERS.cornerRefinementMinAccuracy = 0.5

# Read camera calibration params. The calibration parameters are
# camera model specific. These calibration params have been made for
# Rapsberry Pi Camera Module 2 but they seem to work OK with a webcam too.
# To create your own calibration params see this guide:
# https://github.com/robot-uprising-hq/ai-backend-connector/blob/master/docs/Camera-Calibration.md
with open('rpi-camera-calib-params.json') as json_file:
    calib_params = json.load(json_file)
    MTX = np.array(calib_params['mtx'], dtype=np.float32)
    DIST = np.array(calib_params['dist'], dtype=np.float32)
SIZE_OF_MARKER = 0.15


def print_transforms(transforms):
    """
    Function to pretty print the Aruco marker ID, X and Y coordinate
    and rotation of the robots found.
    """
    for aruco_id in transforms.keys():
        position = transforms[aruco_id]['position']
        rotation = transforms[aruco_id]['rotation']

        print(f'=== Aruco {aruco_id}\n'
              f'Position: X: {position[0]:.2f}, Y: {position[1]:.2f}\n'
              f'Rotation: {rotation[0]:.2f} Degrees\n')


def get_robot_positions(capture):
    ret, frame = capture.read()
    if frame is None:
        return None

    corners, detected_ids, rejected_img_points = \
        aruco.detectMarkers(frame,
                            ARUCO_DICT,
                            parameters=ARUCO_DETECTER_PARAMETERS)

    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners,
                                                        SIZE_OF_MARKER,
                                                        MTX,
                                                        DIST)

    if tvecs is not None and rvecs is not None:
        imaxis = aruco.drawDetectedMarkers(frame, corners, detected_ids)
        for i, _ in enumerate(tvecs):
            aruco.drawAxis(imaxis,
                            MTX,
                            DIST,
                            rvecs[i],
                            tvecs[i],
                            SIZE_OF_MARKER)
        transforms = aruco_poses_to_transforms(detected_ids=detected_ids,
                                                corners=corners,
                                                rvecs=rvecs)
        #print_transforms(transforms)

        return transforms
    return None
