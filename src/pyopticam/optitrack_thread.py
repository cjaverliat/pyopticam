import threading

import numpy as np

from pyopticam import OptitrackStream


class OptitrackThread(threading.Thread):
    """Background thread that continuously reads from an OptitrackStream.

    Call read() at any time to get the most recent synchronized frame without
    blocking the caller.
    """

    def __init__(self, cam_serials: list[int], timeout_ms: int = 100) -> None:
        super().__init__(daemon=True)
        self._stream = OptitrackStream(cam_serials, timeout_ms)
        self._frames: np.ndarray | None = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

    @property
    def cameras(self):
        return self._stream.cameras

    def read(self) -> np.ndarray | None:
        """Return the most recent frame (non-blocking), or None if none yet."""
        with self._lock:
            return self._frames

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        while not self._stop_event.is_set():
            ok, frame = self._stream.read()
            if ok:
                with self._lock:
                    self._frames = frame
        self._stream.release()
