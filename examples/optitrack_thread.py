import numpy as np
import cv2
import threading
import time
import pyopticam as m


def configure_camera(camera, mode, exposure=3000, framerate=120, delay_strobe=False, index=0):
    """Configure and start a camera.

    Mirrors the C++ ConfigureCamera sample from the OptiTrack SDK.
    Sets video mode, exposure, gain, LEDs, and starts output.
    """
    camera.SetVideoType(mode)
    camera.SetAEC(False)
    camera.SetAGC(False)
    camera.SetCameraResolution(0)
    camera.SetFrameRate(framerate)
    camera.SetExposure(exposure)
    camera.SetMJPEGQuality(0, True)
    camera.SetLED(m.eStatusLEDs.IlluminationLED, True)
    camera.SetIRFilter(True)
    camera.SetImagerGain(m.eImagerGain.Gain_Level7)
    camera.SetTextOverlay(True)
    camera.SetStatusRingRGB(64, 0, 255)
    camera.SetNumeric(True, index + 1)
    if delay_strobe and index > 0:
        camera.SetShutterDelay(int(exposure * 1.2 * index))
        camera.SetStrobeOffset(int(exposure * 1.2 * index))
    camera.Start()
    print(f"  Camera {index} started: {camera.Width()}x{camera.Height()}")


class OptitrackThread(threading.Thread):
    '''A thread for receiving images from the Optitrack SDK'''

    def __init__(self, mode=m.eVideoMode.MJPEGMode, exposure=50, delay_strobe=False, framerate=-1, camera_serials=None, use_sync=False):
        '''Initialize Optitrack Reception.

        camera_serials: list of integer serial numbers to use, e.g. [12345, 67890].
                        If None, falls back to a bandwidth-based limit (3 for MJPEG, 4 for Grayscale).
        '''
        threading.Thread.__init__(self)
        self.should_run = True
        self.deadmansSwitch = time.time()
        self.t = 0
        self.current_frame = None

        self.mode = mode
        self.exposure = exposure
        self.delay_strobe = delay_strobe
        self.framerate = framerate

        m.CameraManager.X().WaitForInitialization()

        requested = set(camera_serials) if camera_serials is not None else None
        if requested:
            print(f"Waiting for camera(s) with serials: {sorted(requested)}...")
        else:
            print("Waiting for camera(s)...")

        # Poll until all requested cameras (or at least one) are fully initialized.
        while True:
            time.sleep(0.5)
            camera_list = m.CameraList()
            camera_list.Refresh()
            entries = [camera_list.get(i) for i in range(camera_list.Count())]

            if requested is not None:
                initialized = {e.Serial() for e in entries
                               if e.Serial() in requested and e.State() == m.eCameraState.Initialized}
                if initialized == requested:
                    break
                missing = requested - initialized
                print(f"  Still waiting for serials: {sorted(missing)}")
            else:
                initialized = {e.Serial() for e in entries if e.State() == m.eCameraState.Initialized}
                if initialized:
                    break
                print(f"  No cameras initialized yet ({len(entries)} found)")

        # Select cameras to use, preserving the order of the provided list.
        entries_by_serial = {e.Serial(): e for e in entries}
        if requested is not None:
            selected_entries = [entries_by_serial[s] for s in camera_serials if s in entries_by_serial]
        else:
            # Bandwidth-based fallback: 4 for Grayscale, 3 for MJPEG on a 1 GbE switch
            limit = 4 if self.mode == m.eVideoMode.GrayscaleMode else 3
            selected_entries = sorted(
                (e for e in entries if e.State() == m.eCameraState.Initialized),
                key=lambda e: e.Serial()
            )[:limit]

        self.camera_array = []
        self.camera_entries_array = selected_entries
        self.camera_serials = []

        for i, entry in enumerate(selected_entries):
            print(f"  Camera {i}: serial={entry.Serial()}  name={entry.Name()}  state={entry.State()}")
            if entry.State() == m.eCameraState.Initialized:
                self.camera_array.append(m.CameraManager.X().GetCamera(entry.UID()))
                self.camera_serials.append(entry.Serial())
                time.sleep(0.05)

        if self.mode != m.eVideoMode.GrayscaleMode and use_sync:
            print("Creating Sync Object...")
            self.sync = m.cModuleSync.Create()
            for cam in self.camera_array:
                self.sync.AddCamera(cam)
            print(f"  Frame delivery rate: {self.sync.FrameDeliveryRate()}")
        else:
            self.sync = None

        for i, cam in enumerate(self.camera_array):
            configure_camera(cam, self.mode, self.exposure, self.framerate, self.delay_strobe, i)
            time.sleep(0.05)

        self.camera_serials = np.array(self.camera_serials, dtype=np.int32).reshape(-1, 1)
        print(f"Ready with {len(self.camera_array)} camera(s), serials: {self.camera_serials}")

        self.newFrame = False

    def fetch_frame(self):
        if self.mode == m.eVideoMode.GrayscaleMode:
            time.sleep(0.15)  # This sleep appears to be necessary to allow the cameras to retrieve frames
            new_frame = m.GetSlowFrameArray(self.camera_serials)
        elif self.sync is None:
            # No sync object: fetch each camera's latest frame independently with its own camera context
            new_frame = m.GetFrameArrayNoSync(self.camera_array)
        elif self.mode == m.eVideoMode.ObjectMode:
            new_frame = m.GetFrameGroupObjectArray(self.sync)
            new_frame = np.nan_to_num(new_frame, nan=0.0)
        else:
            current_framegroup = m.GetFrameGroup(self.sync, 100)
            if current_framegroup is None or current_framegroup.Count() == 0:
                print("[DEBUG] GetFrameGroup timed out (no frame)", flush=True)
                return  # timeout, no frame available; caller checks should_run
            print(f"[DEBUG] Got frame group with {current_framegroup.Count()} camera(s)", flush=True)
            new_frame = np.zeros(
                (current_framegroup.Count(), self.camera_array[0].Height(), self.camera_array[0].Width()),
                dtype=np.uint8)
            m.FillTensorFromFrameGroup(self.camera_array[0], current_framegroup, new_frame)
        self.current_frame = new_frame
        self.newFrame = True

    def run(self):
        print("Beginning Optitrack Receive Thread!")
        try:
            while self.should_run:
                self.fetch_frame()
        except Exception as e:
            import traceback
            print(f"[ERROR] Optitrack thread crashed: {e}")
            traceback.print_exc()
        finally:
            if self.sync:
                self.sync.RemoveAllCameras()
                m.cModuleSync.Destroy(self.sync)
            for i in range(len(self.camera_array)):
                if self.camera_array[i].State() == m.eCameraState.Initialized:
                    print(f"Stopping Camera {i}...")
            m.CameraManager.X().Shutdown()
            print("Exiting Optitrack Camera Thread!")

    def read(self):
        '''Retrieves the most recent frame from the system'''
        self.newFrame = False
        self.deadmansSwitch = time.time()
        return self.current_frame

    def stop(self):
        '''Ends the video receiving thread'''
        self.should_run = False
