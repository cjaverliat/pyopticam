import time
from pathlib import Path

import cv2
import numpy as np

import pyopticam as m
from pyopticam import read_mcal, optitrack_thread

calib_path = Path("./assets/example_calib.mcal")
calib = read_mcal(calib_path)
cam_serials = list(calib.keys())

optitrack = optitrack_thread.OptitrackThread(cam_serials=cam_serials)
optitrack.start()

for camera in optitrack.cameras:
    camera.SetAEC(False)
    camera.SetAGC(False)
    camera.SetFrameRate(120)
    camera.SetExposure(6000)
    camera.SetMJPEGQuality(0, True)
    camera.SetLED(m.eStatusLEDs.IlluminationLED, True)
    camera.SetIRFilter(True)
    camera.SetImagerGain(m.eImagerGain.Gain_Level7)
    camera.SetStatusRingRGB(64, 0, 255)

print("Starting to retrieve frame groups...")

fps_history = []
last_frame_time = None


def make_mosaic(frames: np.ndarray) -> np.ndarray:
    n = frames.shape[0]
    cols = int(np.ceil(np.sqrt(n)))
    rows = int(np.ceil(n / cols))
    h, w = frames.shape[1], frames.shape[2]
    blank = np.zeros((h, w), dtype=frames.dtype)
    padded = list(frames) + [blank] * (rows * cols - n)
    rows_list = [np.hstack(padded[r * cols:(r + 1) * cols]) for r in range(rows)]
    return np.vstack(rows_list)


try:
    while True:
        frame = optitrack.read()
        if frame is not None:
            now = time.perf_counter()
            if last_frame_time is not None:
                fps_history.append(1.0 / (now - last_frame_time))
                if len(fps_history) > 30:
                    fps_history.pop(0)
            last_frame_time = now

            fps = sum(fps_history) / len(fps_history) if fps_history else 0.0
            mosaic = make_mosaic(frame)
            mosaic = cv2.resize(mosaic, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_NEAREST)
            cv2.putText(mosaic, f"FPS: {fps:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, 255, 2, cv2.LINE_AA)
            cv2.imshow("Frame", mosaic)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    optitrack.stop()
    optitrack.join()
    cv2.destroyAllWindows()
    print("Fin!")
