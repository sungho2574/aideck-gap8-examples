#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Multi-drone viewer for WiFi image streamer.
#
# Architecture: laptop acts as WiFi hotspot (AP), multiple Crazyflies connect
# to it in STA mode. Each Crazyflie runs a TCP server on port 5000.
# This script connects to each drone and displays all streams in a grid.
#
# Usage:
#   # Explicit IPs
#   python multi-viewer.py 192.168.3.2 192.168.3.3
#
#   # Auto-discover on subnet
#   python multi-viewer.py --discover --subnet 192.168.3.0/24
#
#   # Both: auto-discover + extra IPs
#   python multi-viewer.py --discover --subnet 192.168.3.0/24 192.168.3.10

import argparse
import ipaddress
import socket
import struct
import threading
from concurrent.futures import ThreadPoolExecutor
from math import ceil, sqrt

import cv2
import numpy as np

PORT = 5000
CAM_WIDTH = 324
CAM_HEIGHT = 244


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


def try_connect(ip, port=PORT, timeout=0.5):
    """Try connecting to ip:port. Returns (ip, socket) or None."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect((ip, port))
        s.settimeout(None)
        return (ip, s)
    except Exception:
        return None


def discover(subnet, port=PORT, workers=64):
    """Scan subnet in parallel for open port. Returns list of (ip, socket)."""
    print(f"Scanning {subnet} for Crazyflies on port {port}...")
    hosts = list(ipaddress.ip_network(subnet, strict=False).hosts())
    found = []
    with ThreadPoolExecutor(max_workers=workers) as ex:
        results = list(ex.map(lambda ip: try_connect(str(ip), port, 0.5), hosts))
    for r in results:
        if r is not None:
            print(f"  Found Crazyflie at {r[0]}")
            found.append(r)
    if not found:
        print("  No Crazyflies found on subnet.")
    return found


def make_grid(frames_with_labels, cell_w=CAM_WIDTH, cell_h=CAM_HEIGHT):
    """Arrange (label, frame_or_None) pairs into a grid image."""
    n = len(frames_with_labels)
    if n == 0:
        return np.zeros((cell_h, cell_w, 3), dtype=np.uint8)

    cols = max(1, ceil(sqrt(n)))
    rows = ceil(n / cols)
    blank = np.zeros((cell_h, cell_w, 3), dtype=np.uint8)
    grid = np.zeros((rows * cell_h, cols * cell_w, 3), dtype=np.uint8)

    for i, (label, frame) in enumerate(frames_with_labels):
        r, c = divmod(i, cols)
        cell = blank.copy() if frame is None else frame.copy()
        if cell.shape[:2] != (cell_h, cell_w):
            cell = cv2.resize(cell, (cell_w, cell_h))
        cv2.putText(cell, label, (5, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0), 1, cv2.LINE_AA)
        grid[r * cell_h:(r + 1) * cell_h, c * cell_w:(c + 1) * cell_w] = cell

    return grid


class DroneReceiver:
    def __init__(self, ip, sock):
        self.ip = ip
        self.sock = sock
        self.frame = None
        self.connected = True
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while self.connected:
            try:
                frame = receive_frame(self.sock)
                if frame is not None:
                    with self._lock:
                        self.frame = frame
            except Exception as e:
                print(f"[{self.ip}] disconnected: {e}")
                with self._lock:
                    self.connected = False

    def get_frame(self):
        with self._lock:
            return self.frame

    def stop(self):
        self.connected = False
        try:
            self.sock.close()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(
        description='Multi-drone Crazyflie WiFi stream viewer')
    parser.add_argument('ips', nargs='*', metavar='IP',
                        help='Crazyflie IP address(es)')
    parser.add_argument('-p', '--port', type=int, default=PORT,
                        help=f'TCP port (default {PORT})')
    parser.add_argument('--discover', action='store_true',
                        help='Auto-discover Crazyflies by scanning subnet')
    parser.add_argument('--subnet', default='192.168.3.0/24',
                        help='Subnet to scan (default: 192.168.3.0/24)')
    args = parser.parse_args()

    connections = []

    if args.discover:
        connections = discover(args.subnet, args.port)

    already_found = {ip for ip, _ in connections}
    for ip in args.ips:
        if ip in already_found:
            continue
        print(f"Connecting to {ip}:{args.port}...")
        result = try_connect(ip, args.port, timeout=5.0)
        if result:
            print(f"  Connected.")
            connections.append(result)
        else:
            print(f"  Could not connect to {ip}:{args.port}")

    if not connections:
        print("No Crazyflies to connect to. Exiting.")
        return

    print(f"\nStreaming from {len(connections)} Crazyflie(s). Press ESC to quit.\n")
    receivers = [DroneReceiver(ip, sock) for ip, sock in connections]

    try:
        while True:
            frames_with_labels = [(r.ip, r.get_frame()) for r in receivers]
            grid = make_grid(frames_with_labels)
            cv2.imshow('Crazyflie Multi-View', grid)
            if cv2.waitKey(1) == 27:  # ESC
                break
    finally:
        for r in receivers:
            r.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
