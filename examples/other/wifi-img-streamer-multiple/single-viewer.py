#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Single-drone viewer for WiFi image streamer.
#
# Usage:
#   python single-viewer.py 172.20.10.2
#   python single-viewer.py --discover --subnet 172.20.10.0/24

import argparse
import ipaddress
import socket
import struct
import threading
from concurrent.futures import ThreadPoolExecutor

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
    try:
        packetInfoRaw = rx_bytes(sock, 4)
        print(f"[DEBUG] Got packet header: {len(packetInfoRaw)} bytes")
        [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
        print(f"[DEBUG] length={length}, routing={routing}, function={function}")

        imgHeader = rx_bytes(sock, length - 2)
        print(f"[DEBUG] Got image header: {len(imgHeader)} bytes")
        [magic, width, height, depth, fmt, size] = struct.unpack('<BHHBBI', imgHeader)
        print(f"[DEBUG] magic={hex(magic)}, w={width}, h={height}, fmt={fmt}, size={size}")

        if magic != 0xBC:
            print(f"[DEBUG] Bad magic: {hex(magic)}, expected 0xBC")
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
                print("[DEBUG] JPEG decode failed")
                return None
            if len(img.shape) == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        print(f"[DEBUG] Frame received: {img.shape}")
        return img
    except Exception as e:
        print(f"[DEBUG] receive_frame exception: {e}")
        raise


def try_connect(ip, port=PORT, timeout=0.5):
    """Try connecting to ip:port. Returns socket or None."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect((ip, port))
        s.settimeout(None)
        return s
    except Exception:
        return None


def discover(subnet, port=PORT, workers=64):
    """Scan subnet in parallel for open port. Returns first IP found."""
    print(f"Scanning {subnet} for Crazyflies on port {port}...")
    hosts = list(ipaddress.ip_network(subnet, strict=False).hosts())
    with ThreadPoolExecutor(max_workers=workers) as ex:
        results = list(ex.map(lambda ip: try_connect(str(ip), port, 0.5), hosts))
    for ip, s in zip(hosts, results):
        if s is not None:
            print(f"  Found Crazyflie at {ip}")
            return str(ip), s
    print("  No Crazyflies found on subnet.")
    return None, None


class DroneReceiver:
    def __init__(self, ip, sock):
        self.ip = ip
        self.sock = sock
        self.frame = None
        self.connected = True
        self.fps = 0
        self.frame_count = 0
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        import time
        start_time = time.time()
        frame_attempt = 0
        while self.connected:
            try:
                frame_attempt += 1
                if frame_attempt % 100 == 0:
                    print(f"[{self.ip}] Received {self.frame_count} frames so far...")
                frame = receive_frame(self.sock)
                if frame is not None:
                    with self._lock:
                        self.frame = frame
                        self.frame_count += 1
                        elapsed = time.time() - start_time
                        if elapsed > 0:
                            self.fps = self.frame_count / elapsed
                        if self.frame_count == 1:
                            print(f"[{self.ip}] Got first frame! Shape: {frame.shape}")
                else:
                    print(f"[{self.ip}] Received None frame")
            except Exception as e:
                print(f"[{self.ip}] Error receiving frame: {e}")
                with self._lock:
                    self.connected = False

    def get_frame(self):
        with self._lock:
            return self.frame, self.fps

    def stop(self):
        self.connected = False
        try:
            self.sock.close()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(
        description='Single-drone Crazyflie WiFi stream viewer')
    parser.add_argument('ip', nargs='?', metavar='IP',
                        help='Crazyflie IP address')
    parser.add_argument('-p', '--port', type=int, default=PORT,
                        help=f'TCP port (default {PORT})')
    parser.add_argument('--discover', action='store_true',
                        help='Auto-discover Crazyflie by scanning subnet')
    parser.add_argument('--subnet', default='172.20.10.0/24',
                        help='Subnet to scan (default: 172.20.10.0/24)')
    args = parser.parse_args()

    ip = None
    sock = None

    if args.discover:
        ip, sock = discover(args.subnet, args.port)
    elif args.ip:
        print(f"Connecting to {args.ip}:{args.port}...")
        sock = try_connect(args.ip, args.port, timeout=5.0)
        if sock:
            print(f"  Connected.")
            ip = args.ip
        else:
            print(f"  Could not connect to {args.ip}:{args.port}")

    if ip is None or sock is None:
        print("No Crazyflie to connect to. Exiting.")
        return

    print(f"\nStreaming from {ip}. Press ESC to quit.\n")
    receiver = DroneReceiver(ip, sock)

    try:
        while True:
            frame, fps = receiver.get_frame()
            if frame is not None:
                display_frame = frame.copy()
                cv2.putText(display_frame, f"{ip} | FPS: {fps:.1f}",
                            (5, 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 1, cv2.LINE_AA)
                cv2.imshow(f'Crazyflie - {ip}', display_frame)
            if cv2.waitKey(1) == 27:  # ESC
                break
    finally:
        receiver.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
