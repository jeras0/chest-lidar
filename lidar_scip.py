"""Minimal Hokuyo UST-10LX driver (SCIP 2.0 over TCP). Python stdlib only.

UST-10LX facts: 270 deg field of view, 0.25 deg steps, steps 0..1080,
step 540 points straight ahead, range 0.06-10 m, 40 Hz scans.
Default address 192.168.0.10, port 10940.
"""
import math
import random
import socket
import time

FIRST_STEP = 0
LAST_STEP = 1080
FRONT_STEP = 540
STEP_RAD = math.radians(0.25)


def step_to_angle(step):
    """Angle in radians, 0 = straight ahead, positive = to the left (counter-clockwise)."""
    return (step - FRONT_STEP) * STEP_RAD


def decode(data):
    """SCIP 3-character encoding: each char minus 0x30 gives 6 bits."""
    out = []
    for i in range(0, len(data) - 2, 3):
        v = 0
        for c in data[i:i + 3]:
            v = (v << 6) | (ord(c) - 0x30)
        out.append(v)
    return out


class Lidar:
    def __init__(self, host="192.168.0.10", port=10940, timeout=2.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.buf = b""
        self._cmd("QT")  # stop anything a previous run left going
        self._cmd("BM")  # laser on

    def _read_block(self):
        # every SCIP reply ends with an empty line ("\n\n")
        while b"\n\n" not in self.buf:
            chunk = self.sock.recv(8192)
            if not chunk:
                raise ConnectionError("lidar closed the connection")
            self.buf += chunk
        block, self.buf = self.buf.split(b"\n\n", 1)
        return block.decode("ascii").split("\n")

    def _cmd(self, cmd):
        self.sock.sendall((cmd + "\n").encode("ascii"))
        return self._read_block()

    def scan(self):
        """Return one scan as a list of 1081 distances in millimetres.
        Values below 20 mean "no reading" (error codes); treat them as invalid."""
        lines = self._cmd("GD%04d%04d01" % (FIRST_STEP, LAST_STEP))
        status = lines[1][:2]
        if status != "00":
            raise RuntimeError("lidar error status %r" % status)
        # lines[2] is the timestamp; data lines carry a trailing checksum char
        data = "".join(line[:-1] for line in lines[3:])
        return decode(data)

    def close(self):
        try:
            self.sock.sendall(b"QT\n")
        finally:
            self.sock.close()


class FakeLidar:
    """Simulated room with a wall that moves closer and further, for testing
    without hardware (run with --sim)."""

    def __init__(self):
        self.t0 = time.time()

    def scan(self):
        time.sleep(1 / 40)
        t = time.time() - self.t0
        wall = 2.0 + 1.6 * math.sin(t * 0.5)  # metres in front, 0.4..3.6
        out = []
        for s in range(FIRST_STEP, LAST_STEP + 1):
            a = step_to_angle(s)
            c = math.cos(a)
            d = wall / c if c > 0.05 else 10.0  # flat wall ahead
            side = 1.5 / abs(math.sin(a)) if abs(math.sin(a)) > 0.05 else 10.0
            d = min(d, side, 10.0)  # corridor walls left/right
            out.append(int(d * 1000 + random.uniform(-10, 10)))
        return out

    def close(self):
        pass
