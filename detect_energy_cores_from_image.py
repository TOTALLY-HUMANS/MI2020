"""
Example script to detect objects by color from an image using OpenCV.
"""
import numpy as np
import cv2
from utils.ecore_utils import image_to_center_points


# Select the camera source by setting this
VIDEO_SOURCE = "ffmpeg"  # Options: 'gstreamer', 'webcam' or 'opencv'

# Low and High values in HSV-colorspace for detecting color range.
# See the these articles to understand more about HSV-colorspace:
# https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html
# https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html
# https://en.wikipedia.org/wiki/HSL_and_HSV
POS_ECORE_LOW_COLOR = np.array([20, 80, 80], dtype=np.float32)
POS_ECORE_HIGH_COLOR = np.array([60, 255, 255], dtype=np.float32)
NEG_ECORE_LOW_COLOR = np.array([150, 80, 80], dtype=np.float32)
NEG_ECORE_HIGH_COLOR = np.array([179, 255, 255], dtype=np.float32)


def print_core_positions(pos_ecore_positions, neg_ecore_positions):
    """
    Function to pretty print the X and Y coordinates for energy cores
    """
    if pos_ecore_positions:
        for i, core in enumerate(pos_ecore_positions):
            print(f'Positive Core {i}: X: {core[0]:.2f}, Y: {core[1]:.2f}')
    if neg_ecore_positions:
        for i, core in enumerate(neg_ecore_positions):
            print(f'Negative Core {i}: X: {core[0]:.2f}, Y: {core[1]:.2f}')
    if not pos_ecore_positions and not neg_ecore_positions:
        print('No Energy Cores detected')
    print('=== Done\n')


def get_core_positions(capture):
    ret, frame = capture.read()
    if frame is None:
        return None

    pos_ecore_positions = image_to_center_points(
        frame,
        POS_ECORE_LOW_COLOR,
        POS_ECORE_HIGH_COLOR,
        'Positive Energy Cores')
    neg_ecore_positions = image_to_center_points(
        frame,
        NEG_ECORE_LOW_COLOR,
        NEG_ECORE_HIGH_COLOR,
        'Negative Energy Cores')
    
    
    #print_core_positions(pos_ecore_positions, neg_ecore_positions)
    return pos_ecore_positions, neg_ecore_positions