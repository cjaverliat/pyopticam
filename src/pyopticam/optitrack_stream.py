import time

import numpy as np

import pyopticam as m


class OptitrackStream:
    """Synchronized multi-camera MJPEG frame reader.

    Mimics the cv2.VideoCapture interface: construct to open, call read() in a
    loop, call release() (or use as a context manager) when done.
    """

    def __init__(self, cam_ids: list[int], timeout_ms: int = 100) -> None:
        """Wait for all cameras in cam_ids to initialize, then start MJPEG capture.

        Args:
            cam_ids:     Ordered list of camera IDs to open.
            timeout_ms:  Per-call timeout passed to GetFrameGroup.
        """
        m.CameraManager.X().WaitForInitialization()
        self._cameras: list[m.Camera] = self._wait_for_cameras(cam_ids)
        self._timeout_ms = timeout_ms

        for cam in self._cameras:
            cam.SetVideoType(m.eVideoMode.MJPEGMode)
            cam.Start()

        self._sync = m.cModuleSync.Create()
        for cam in self._cameras:
            self._sync.AddCamera(cam)

        self._frame_shape = (
            len(self._cameras),
            self._cameras[0].Height(),
            self._cameras[0].Width(),
        )

    @property
    def cameras(self) -> list[m.Camera]:
        """Camera objects in the order supplied to the constructor."""
        return self._cameras

    def read(self) -> tuple[bool, np.ndarray | None]:
        """Block until a synchronized frame group is available.

        Returns:
            (True,  ndarray[N, H, W] uint8) on success.
            (False, None)                   on timeout.
        """
        frame_group = m.GetFrameGroup(self._sync, self._timeout_ms)
        if frame_group is None or frame_group.Count() == 0:
            return False, None
        frames = np.empty(self._frame_shape, dtype=np.uint8)
        m.FillTensorFromFrameGroup(self._cameras[0], frame_group, frames)
        return True, frames

    def release(self) -> None:
        """Destroy the sync object and shut down the camera manager."""
        if self._sync is not None:
            self._sync.RemoveAllCameras()
            m.cModuleSync.Destroy(self._sync)
            self._sync = None
        m.CameraManager.X().Shutdown()

    def __enter__(self) -> "OptitrackStream":
        return self

    def __exit__(self, *_) -> None:
        self.release()

    def _wait_for_cameras(self, cam_ids: list[int]) -> list[m.Camera]:
        requested = set(cam_ids)
        print(f"Waiting for cameras {sorted(requested)}...")
        while True:
            camera_list = m.CameraList()
            camera_list.Refresh()
            entries = [camera_list.get(i) for i in range(camera_list.Count())]

            ready: dict[int, m.Camera] = {}
            for entry in entries:
                cam = m.CameraManager.X().GetCameraBySerial(entry.Serial())
                if cam is None or cam.CameraID() not in requested:
                    continue
                if entry.State() == m.eCameraState.Initialized:
                    ready[cam.CameraID()] = cam

            if set(ready) == requested:
                return [ready[cid] for cid in cam_ids]

            print(f"  Still waiting for camera IDs: {sorted(requested - set(ready))}")
            time.sleep(0.5)
