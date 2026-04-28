#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#  Based on opencv-viewer.py
#  Adds ArUco marker detection and overlay to the streamed images.

import cv2
import argparse
import time
import socket
import struct
import numpy as np
import os

# Hardcoded default calibration for the AI-deck monochrome camera.
# These are approximate intrinsics suitable for marker overlay and pose visualization.
CAMERA_MATRIX = np.array([
    [320.0,   0.0, 162.0],
    [0.0, 320.0, 122.0],
    [0.0,   0.0,   1.0],
], dtype=np.float32)
DIST_COEFFS = np.zeros((5, 1), dtype=np.float32)
MARKER_SIZE_M = 0.07

parser = argparse.ArgumentParser(description='Connect to AI-deck JPEG streamer example with ArUco overlay')
parser.add_argument("-n",  default="192.168.4.1", metavar="ip", help="AI-deck IP")
parser.add_argument("-p", type=int, default=5000, metavar="port", help="AI-deck port")
parser.add_argument('--save', action='store_true', help="Save streamed images")
parser.add_argument('--dict', default='4X4_50', help="ArUco dictionary (e.g. 4X4_50, 5X5_100)")
args = parser.parse_args()

deck_port = args.p
deck_ip = args.n

print("Connecting to socket on {}:{}...".format(deck_ip, deck_port))
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((deck_ip, deck_port))
print("Socket connected")


def rx_bytes(size):
    data = bytearray()
    while len(data) < size:
        chunk = client_socket.recv(size - len(data))
        if not chunk:
            raise ConnectionError('Socket closed')
        data.extend(chunk)
    return data


# Setup ArUco dictionary and detector parameters with compatibility fallbacks
try:
    aruco = cv2.aruco
except AttributeError:
    raise RuntimeError('OpenCV was built without aruco module')

DICT_MAP = {
    '4X4_50': aruco.DICT_4X4_50,
    '4X4_100': aruco.DICT_4X4_100,
    '5X5_50': aruco.DICT_5X5_50,
    '5X5_100': aruco.DICT_5X5_100,
}

aruco_key = args.dict if args.dict in DICT_MAP else '4X4_50'
ARUCO_DICT = aruco.getPredefinedDictionary(DICT_MAP[aruco_key])
aruco_params = aruco.DetectorParameters()

start = time.time()
count = 0

os.makedirs('stream_out/raw', exist_ok=True)
os.makedirs('stream_out/debayer', exist_ok=True)
os.makedirs('stream_out/aruco', exist_ok=True)

while True:
    packetInfoRaw = rx_bytes(4)
    [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
    imgHeader = rx_bytes(length - 2)
    [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

    if magic != 0xBC:
        continue

    imgStream = bytearray()
    while len(imgStream) < size:
        packetInfoRaw = rx_bytes(4)
        [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
        chunk = rx_bytes(length - 2)
        imgStream.extend(chunk)

    count += 1
    meanTimePerImage = (time.time() - start) / count
    print(f"Mean time/image: {meanTimePerImage:.3f}s  FPS: {1.0/meanTimePerImage:.2f}")

    # Treat all incoming frames as grayscale (camera is monochrome)
    if format == 0:
        # Raw grayscale image (no debayer/color processing)
        gray = np.frombuffer(imgStream, dtype=np.uint8)
        try:
            gray.shape = (244, 324)
        except Exception:
            gray = gray.reshape((height, width))
        if args.save:
            cv2.imwrite(f"stream_out/raw/img_{count:06d}.png", gray)
    else:
        # JPEG image: decode, then convert to grayscale if needed
        nparr = np.frombuffer(imgStream, np.uint8)
        decoded = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)
        if decoded is None:
            continue
        if decoded.ndim == 2:
            gray = decoded
        elif decoded.ndim == 3 and decoded.shape[2] == 4:
            gray = cv2.cvtColor(decoded, cv2.COLOR_BGRA2GRAY)
        else:
            gray = cv2.cvtColor(decoded, cv2.COLOR_BGR2GRAY)
        if args.save:
            cv2.imwrite(f"stream_out/aruco/img_{count:06d}.png", gray)

    # Visualization image (single-channel). Draw markers in white (255).
    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    # Detection: use ArucoDetector if available, otherwise fallback to detectMarkers
    try:
        if hasattr(aruco, 'ArucoDetector'):
            detector = aruco.ArucoDetector(
                ARUCO_DICT, aruco_params) if aruco_params is not None else aruco.ArucoDetector(ARUCO_DICT)
            corners, ids, rejected = detector.detectMarkers(gray)
        else:
            # older API
            if aruco_params is not None:
                corners, ids, rejected = aruco.detectMarkers(gray, ARUCO_DICT, parameters=aruco_params)
            else:
                corners, ids, rejected = aruco.detectMarkers(gray, ARUCO_DICT)
    except Exception:
        # As a last resort, set empty results
        corners, ids, rejected = [], None, None

    if corners is not None and len(corners) > 0:
        cv2.aruco.drawDetectedMarkers(vis, corners, ids)

        for idx, c in enumerate(corners):
            # Pose estimation from marker corners
            half_size = MARKER_SIZE_M / 2.0
            obj_points = np.array([
                [-half_size,  half_size, 0.0],
                [half_size,  half_size, 0.0],
                [half_size, -half_size, 0.0],
                [-half_size, -half_size, 0.0],
            ], dtype=np.float32)

            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                c,
                CAMERA_MATRIX,
                DIST_COEFFS,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not success:
                continue

            cv2.drawFrameAxes(vis, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, MARKER_SIZE_M / 2.0)

    cv2.imshow('ARUCO', vis)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

client_socket.close()
