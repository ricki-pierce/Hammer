"""
imu_client.py  --  put this file in the SAME folder as the master script.

Talks to the C# AwindaMarkerRecorder over localhost TCP (line protocol, tab-separated):

    START                         -> "OK" / "ERR <msg>"   start IMU measurement/recording
    MARK <code> <label> <py_ns>   (no reply)              stamp a marker on the IMU data
    COND <name>                   (no reply)              set condition name for following rows
    SAVE                          -> "OK <path>" / "ERR"  stop + write file to IMUData folder

If the C# app is not reachable, the IMU is disabled and the rest of the experiment still runs.
"""

import socket
import threading
import time

IMU_HOST = "127.0.0.1"
IMU_PORT = 5005


class IMUClient:
    def __init__(self, host=IMU_HOST, port=IMU_PORT):
        self.host, self.port = host, port
        self.enabled = False
        self._sock = None
        self._lock = threading.Lock()
        self._buf = b""

    # ---------- helpers ----------
    @staticmethod
    def _clean(text):
        return str(text).replace("\t", " ").replace("\r", " ").replace("\n", " ")

    def _send_line(self, line):
        self._sock.sendall((line + "\n").encode("utf-8"))

    def _read_line(self, timeout):
        self._sock.settimeout(timeout)
        while b"\n" not in self._buf:
            chunk = self._sock.recv(4096)
            if not chunk:
                raise ConnectionError("IMU app closed the connection")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return line.decode("utf-8").strip()

    def _request(self, line, timeout):
        with self._lock:
            self._send_line(line)
            return self._read_line(timeout)

    def _disable(self, why):
        print(f"IMU DISABLED: {why}")
        self.enabled = False

    # ---------- public API ----------
    def connect(self, timeout=5.0):
        try:
            self._sock = socket.create_connection((self.host, self.port), timeout=timeout)
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.enabled = True
            print(f"IMU app connected on {self.host}:{self.port}")
        except OSError as e:
            self._disable(f"could not connect to C# IMU app ({e}). Is it running?")
        return self.enabled

    def start(self):
        if not self.enabled:
            return False
        try:
            reply = self._request("START", timeout=120.0)
            print(f"IMU START -> {reply}")
            if not reply.startswith("OK"):
                self._disable(reply)
            return self.enabled
        except Exception as e:
            self._disable(f"START failed: {e}")
            return False

    def mark(self, code, label=""):
        """Fire-and-forget; never blocks or raises into the trial loop."""
        if not self.enabled:
            return
        try:
            with self._lock:
                self._send_line(f"MARK\t{int(code)}\t{self._clean(label)}\t{time.time_ns()}")
        except Exception as e:
            self._disable(f"MARK failed: {e}")

    def set_condition(self, name):
        if not self.enabled:
            return
        try:
            with self._lock:
                self._send_line(f"COND\t{self._clean(name)}")
        except Exception as e:
            self._disable(f"COND failed: {e}")

    def stop_and_save(self):
        if not self.enabled:
            return None
        try:
            reply = self._request("SAVE", timeout=120.0)
            print(f"IMU SAVE -> {reply}")
            return reply
        except Exception as e:
            print(f"IMU save failed: {e}")
            return None

    def close(self):
        try:
            if self._sock:
                self._sock.close()
        except Exception:
            pass
        self.enabled = False
