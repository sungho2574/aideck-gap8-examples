#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Drone monitoring dashboard.
# Streams WiFi video from 1-3 Crazyflie drones, detects ArUco markers,
# and displays drone positions + marker positions on a Flask web dashboard.
#
# Usage:
#   python dashboard.py [--config drones.yaml] [--port 5001]

import argparse
import socket
import struct
import threading
import time

import cflib.crtp
import cv2
import numpy as np
import yaml
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from flask import Flask, Response, jsonify, render_template

# ── WiFi streaming constants ─────────────────────────────────────────────────
WIFI_PORT = 5000
CAM_WIDTH = 324
CAM_HEIGHT = 244

# ── ArUco constants ───────────────────────────────────────────────────────────
CAMERA_MATRIX = np.array(
    [[320.0, 0.0, 162.0],
     [0.0, 320.0, 122.0],
     [0.0, 0.0, 1.0]],
    dtype=np.float32,
)
DIST_COEFFS = np.zeros((5, 1), dtype=np.float32)
MARKER_SIZE_M = 0.14

# Precomputed: R_tilt_Y(45°) @ R_cam_mount
# Transforms tvec from OpenCV camera frame to Crazyflie body frame (X-fwd, Y-left, Z-up)
# assuming camera is mounted pointing 45° nose-down from drone front.
_S = np.sqrt(2) / 2
R_CAM_TO_BODY = np.array(
    [[0.0, -_S,  _S],
     [-1.0, 0.0, 0.0],
     [0.0, -_S, -_S]],
    dtype=np.float64,
)

MARKER_TTL = 5.0  # seconds before stale marker is removed


# ── WiFi frame protocol (reused from single-viewer.py) ───────────────────────

def rx_bytes(sock, size):
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("socket closed")
        data.extend(chunk)
    return data


def receive_frame(sock):
    """Receive one frame from a Crazyflie. Returns BGR numpy array."""
    packetInfoRaw = rx_bytes(sock, 4)
    [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)

    imgHeader = rx_bytes(sock, length - 2)
    [magic, width, height, depth, fmt, size] = struct.unpack('<BHHBBI', imgHeader)

    if magic != 0xBC:
        return None

    imgStream = bytearray()
    while len(imgStream) < size:
        packetInfoRaw = rx_bytes(sock, 4)
        [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
        chunk = rx_bytes(sock, length - 2)
        imgStream.extend(chunk)

    if fmt == 0:  # RAW bayer
        img = np.frombuffer(imgStream, dtype=np.uint8).reshape(height, width)
        img = cv2.cvtColor(img, cv2.COLOR_BayerBG2BGR)
    else:  # JPEG
        nparr = np.frombuffer(imgStream, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)
        if img is None:
            return None
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    return img


def try_connect(ip, port=WIFI_PORT, timeout=2.0):
    """Try connecting to ip:port. Returns socket or None."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect((ip, port))
        s.settimeout(None)
        return s
    except Exception:
        return None


# ── DroneReceiver (reused from single-viewer.py) ─────────────────────────────

class DroneReceiver:
    def __init__(self, ip, sock):
        self.ip = ip
        self.sock = sock
        self.frame = None
        self.connected = True
        self.fps = 0.0
        self.frame_count = 0
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        start_time = time.time()
        while self.connected:
            try:
                frame = receive_frame(self.sock)
                if frame is not None:
                    with self._lock:
                        self.frame = frame
                        self.frame_count += 1
                        elapsed = time.time() - start_time
                        if elapsed > 0:
                            self.fps = self.frame_count / elapsed
            except Exception:
                with self._lock:
                    self.connected = False

    def get_frame(self):
        with self._lock:
            return (self.frame.copy() if self.frame is not None else None), self.fps

    def stop(self):
        self.connected = False
        try:
            self.sock.close()
        except Exception:
            pass


# ── ArUco detector ────────────────────────────────────────────────────────────

class ArucoDetector:
    def __init__(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    def detect(self, frame_bgr):
        """Returns list of {id, tvec, rvec, corners}."""
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        results = []
        if ids is None:
            return results

        obj_pts = np.array([
            [-MARKER_SIZE_M / 2,  MARKER_SIZE_M / 2, 0],
            [ MARKER_SIZE_M / 2,  MARKER_SIZE_M / 2, 0],
            [ MARKER_SIZE_M / 2, -MARKER_SIZE_M / 2, 0],
            [-MARKER_SIZE_M / 2, -MARKER_SIZE_M / 2, 0],
        ], dtype=np.float32)

        for i, corner in enumerate(corners):
            img_pts = corner[0].astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts,
                CAMERA_MATRIX, DIST_COEFFS,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not success:
                continue
            if np.linalg.norm(tvec) > 8.0:
                continue
            results.append({
                'id': int(ids[i][0]),
                'tvec': tvec.flatten(),
                'rvec': rvec.flatten(),
                'corners': corner[0],
            })
        return results

    def draw_overlays(self, frame_bgr, detections):
        """Draw marker borders and axes. Returns annotated copy."""
        out = frame_bgr.copy()
        if not detections:
            return out
        corners_list = [np.array([d['corners']]) for d in detections]
        ids_arr = np.array([[d['id']] for d in detections])
        cv2.aruco.drawDetectedMarkers(out, corners_list, ids_arr)
        for d in detections:
            rvec = d['rvec'].reshape(3, 1)
            tvec = d['tvec'].reshape(3, 1)
            cv2.drawFrameAxes(out, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, MARKER_SIZE_M * 0.5)
        return out


# ── Coordinate transform ──────────────────────────────────────────────────────

def transform_to_world(detection, drone_state):
    """Convert ArUco tvec (camera frame) to world XY using drone pose."""
    tvec = detection['tvec'].astype(np.float64)
    p_body = R_CAM_TO_BODY @ tvec

    yaw_rad = np.deg2rad(drone_state.get('yaw', 0.0))
    cy, sy = np.cos(yaw_rad), np.sin(yaw_rad)
    R_yaw = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])

    p_world_rel = R_yaw @ p_body
    return {
        'id': detection['id'],
        'x': float(drone_state.get('x', 0.0) + p_world_rel[0]),
        'y': float(drone_state.get('y', 0.0) + p_world_rel[1]),
        'z': float(drone_state.get('z', 0.0) + p_world_rel[2]),
        'ts': time.time(),
    }


def encode_jpeg(frame_bgr, quality=70):
    ok, buf = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return buf.tobytes()


# ── Shared state ──────────────────────────────────────────────────────────────

class SharedState:
    def __init__(self):
        self._lock = threading.Lock()
        self._drone_states = {}   # drone_id → dict
        self._frames = {}         # drone_id → bytes (JPEG)
        self._markers = {}        # marker_id → dict

    def _init_drone(self, drone_id):
        if drone_id not in self._drone_states:
            self._drone_states[drone_id] = {
                'x': 0.0, 'y': 0.0, 'z': 0.0,
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'wifi_ok': False, 'radio_ok': False, 'fps': 0.0,
            }

    def update_drone_telemetry(self, drone_id, **fields):
        with self._lock:
            self._init_drone(drone_id)
            self._drone_states[drone_id].update(fields)

    def update_frame(self, drone_id, jpeg_bytes):
        with self._lock:
            self._frames[drone_id] = jpeg_bytes

    def update_markers(self, marker_list):
        with self._lock:
            for m in marker_list:
                self._markers[m['id']] = m

    def get_drone_state(self, drone_id):
        with self._lock:
            self._init_drone(drone_id)
            return dict(self._drone_states[drone_id])

    def get_frame(self, drone_id):
        with self._lock:
            return self._frames.get(drone_id)

    def get_api_state(self):
        now = time.time()
        with self._lock:
            drones = [
                {'id': did, **dict(state)}
                for did, state in self._drone_states.items()
            ]
            markers = [
                dict(m)
                for m in self._markers.values()
                if now - m['ts'] < MARKER_TTL
            ]
        return {'drones': drones, 'markers': markers}


# ── DroneWorker (WiFi + ArUco per drone) ─────────────────────────────────────

class DroneWorker:
    def __init__(self, drone_id, wifi_ip, shared_state, aruco_detector):
        self.drone_id = drone_id
        self.wifi_ip = wifi_ip
        self._shared = shared_state
        self._aruco = aruco_detector
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while True:
            sock = try_connect(self.wifi_ip, WIFI_PORT, timeout=3.0)
            if sock is None:
                self._shared.update_drone_telemetry(self.drone_id, wifi_ok=False)
                time.sleep(3.0)
                continue

            self._shared.update_drone_telemetry(self.drone_id, wifi_ok=True)
            receiver = DroneReceiver(self.wifi_ip, sock)
            try:
                self._process_frames(receiver)
            finally:
                receiver.stop()
                self._shared.update_drone_telemetry(self.drone_id, wifi_ok=False)
            time.sleep(1.0)

    def _process_frames(self, receiver):
        while receiver.connected:
            frame, fps = receiver.get_frame()
            if frame is None:
                time.sleep(0.02)
                continue

            detections = self._aruco.detect(frame)

            if detections:
                drone_state = self._shared.get_drone_state(self.drone_id)
                world_markers = [transform_to_world(d, drone_state) for d in detections]
                self._shared.update_markers(world_markers)

            annotated = self._aruco.draw_overlays(frame, detections)
            self._shared.update_frame(self.drone_id, encode_jpeg(annotated))
            self._shared.update_drone_telemetry(self.drone_id, fps=round(fps, 1))
            time.sleep(0.02)


# ── DroneLogger (crazyradio telemetry) ───────────────────────────────────────

class DroneLogger:
    def __init__(self, drone_id, radio_uri, shared_state):
        self.drone_id = drone_id
        self.radio_uri = radio_uri
        self._shared = shared_state
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while True:
            try:
                self._connect_and_log()
            except Exception:
                pass
            self._shared.update_drone_telemetry(self.drone_id, radio_ok=False)
            time.sleep(5.0)

    def _connect_and_log(self):
        cf = Crazyflie(rw_cache='./cache')
        connected_event = threading.Event()
        disconnect_event = threading.Event()

        def _connected(uri):
            lg = LogConfig(name='pose', period_in_ms=100)
            lg.add_variable('stateEstimate.x', 'float')
            lg.add_variable('stateEstimate.y', 'float')
            lg.add_variable('stateEstimate.z', 'float')
            lg.add_variable('stabilizer.roll', 'float')
            lg.add_variable('stabilizer.pitch', 'float')
            lg.add_variable('stabilizer.yaw', 'float')
            cf.log.add_config(lg)
            lg.data_received_cb.add_callback(self._log_data)
            lg.start()
            self._shared.update_drone_telemetry(self.drone_id, radio_ok=True)
            connected_event.set()

        def _disconnected(uri):
            self._shared.update_drone_telemetry(self.drone_id, radio_ok=False)
            disconnect_event.set()

        def _connect_failed(uri, msg):
            disconnect_event.set()

        cf.connected.add_callback(_connected)
        cf.disconnected.add_callback(_disconnected)
        cf.connection_failed.add_callback(_connect_failed)
        cf.open_link(self.radio_uri)

        disconnect_event.wait()
        cf.close_link()

    def _log_data(self, timestamp, data, logconf):
        self._shared.update_drone_telemetry(
            self.drone_id,
            x=data.get('stateEstimate.x', 0.0),
            y=data.get('stateEstimate.y', 0.0),
            z=data.get('stateEstimate.z', 0.0),
            roll=data.get('stabilizer.roll', 0.0),
            pitch=data.get('stabilizer.pitch', 0.0),
            yaw=data.get('stabilizer.yaw', 0.0),
        )


# ── Flask app ─────────────────────────────────────────────────────────────────

app = Flask(__name__)
shared_state = SharedState()
drone_configs = []


@app.route('/')
def index():
    return render_template('index.html', drones=drone_configs)


@app.route('/api/state')
def api_state():
    return jsonify(shared_state.get_api_state())


@app.route('/api/frame/<drone_id>')
def api_frame(drone_id):
    jpeg = shared_state.get_frame(drone_id)
    if jpeg is None:
        return '', 204
    return Response(jpeg, mimetype='image/jpeg')


# ── Entry point ───────────────────────────────────────────────────────────────

def load_config(path):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    return cfg['drones']


def main():
    parser = argparse.ArgumentParser(description='Crazyflie monitoring dashboard')
    parser.add_argument('--config', default='drones.yaml', help='Drone config file')
    parser.add_argument('--port', type=int, default=5001, help='Flask port')
    args = parser.parse_args()

    global drone_configs
    drone_configs = load_config(args.config)

    cflib.crtp.init_drivers()

    aruco_detector = ArucoDetector()

    for d in drone_configs:
        shared_state.update_drone_telemetry(d['id'])
        DroneWorker(d['id'], d['wifi_ip'], shared_state, aruco_detector)
        DroneLogger(d['id'], d['radio_uri'], shared_state)

    print(f"Dashboard running at http://localhost:{args.port}")
    app.run(host='0.0.0.0', port=args.port, threaded=True)


if __name__ == '__main__':
    main()
