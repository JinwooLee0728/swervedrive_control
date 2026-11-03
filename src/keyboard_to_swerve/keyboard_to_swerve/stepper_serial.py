"""Serial thread for the USB stepper controller.

The keyboard loop only posts jog/stop requests. This thread owns the port,
so a blocking read for DONE does not stall swerve commands.
"""

import threading
import time


class StepperSerial(threading.Thread):
    """One Nano stepper. u sends negative steps, n sends positive steps."""

    def __init__(self, port, baud=115200, steps=400, speed=400):
        super().__init__(name="stepper-serial", daemon=True)
        self.port = port
        self.baud = int(baud)
        self.steps = max(1, abs(int(steps)))
        self.speed = min(10000, max(1, int(speed)))
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._request = None
        self._ser = None

    def jog(self, sign):
        sign = 1 if sign >= 0 else -1
        self._post(("jog", sign))

    def stop(self):
        self._post(("stop", 0))

    def shutdown(self):
        self._post(("shutdown", 0))

    def _post(self, request):
        with self._lock:
            self._request = request
        self._wake.set()

    def _take(self):
        with self._lock:
            request = self._request
            self._request = None
        return request

    def run(self):
        try:
            import serial
        except ImportError:
            print("\nstepper: pyserial is not installed", flush=True)
            return

        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except Exception as exc:
            print(f"\nstepper: cannot open {self.port}: {exc}", flush=True)
            return

        try:
            if self._wait_ready():
                self._resync()
                self._loop()
        except Exception as exc:
            print(f"\nstepper: {exc}", flush=True)
        finally:
            try:
                if self._ser is not None and self._ser.is_open:
                    self._ser.close()
            except Exception:
                pass

    def _wait_ready(self):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if self._peek_kind() == "shutdown":
                return False
            line = self._read_line()
            if line is None:
                continue
            print(f"\nstepper: {line}", flush=True)
            if line.startswith("READY"):
                return True
        print("\nstepper: no READY banner; sending commands anyway", flush=True)
        return self._peek_kind() != "shutdown"

    def _resync(self):
        # Opening the port can leave junk in the Nano's line buffer, which
        # makes the first real command come back ERR BAD_COMMAND. Terminate
        # whatever partial line is there and drop the reply.
        self._ser.write(b"\n")
        self._ser.flush()
        time.sleep(0.2)
        self._ser.reset_input_buffer()

    def _peek_kind(self):
        with self._lock:
            if self._request is None:
                return None
            return self._request[0]

    def _loop(self):
        moving = False
        stop_sent = False
        active_sign = 0
        nxt = None

        while True:
            self._wake.clear()
            req = self._take()
            kind = None if req is None else req[0]

            if kind == "shutdown":
                if moving:
                    self._write("STOP")
                    self._read_until_done()
                self._write("ENABLE 0")
                return

            if kind == "stop":
                nxt = None
                if moving and not stop_sent:
                    self._write("STOP")
                    stop_sent = True
                elif not moving:
                    self._write("STOP")

            if kind == "jog":
                sign = req[1]
                if not moving:
                    self._start(sign)
                    moving = True
                    stop_sent = False
                    active_sign = sign
                    nxt = None
                elif sign == active_sign:
                    nxt = sign
                else:
                    nxt = sign
                    if not stop_sent:
                        self._write("STOP")
                        stop_sent = True

            if moving:
                line = self._read_line()
                if line is None:
                    continue
                print(f"\nstepper: {line}", flush=True)
                if line.startswith("DONE"):
                    moving = False
                    stop_sent = False
                    if nxt is not None:
                        sign = nxt
                        nxt = None
                        self._start(sign)
                        moving = True
                        active_sign = sign
                elif line.startswith("ERR") and "BUSY" not in line:
                    moving = False
                    stop_sent = False
                continue

            if req is None:
                self._wake.wait()

    def _start(self, sign):
        steps = self.steps if sign >= 0 else -self.steps
        self._write(f"MOVE {steps} {self.speed}")
        print(f"\nstepper: MOVE {steps} {self.speed}", flush=True)

    def _read_until_done(self):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            line = self._read_line()
            if line is None:
                continue
            print(f"\nstepper: {line}", flush=True)
            if line.startswith("DONE") or line.startswith("ERR"):
                return

    def _write(self, text):
        self._ser.write((text + "\n").encode("ascii"))
        self._ser.flush()

    def _read_line(self):
        raw = self._ser.readline()
        if not raw:
            return None
        return raw.decode("ascii", errors="replace").strip()
