import cv2
import numpy as np
import time
import optitrack_thread
import json
from pathlib import Path

calib = Path("./assets/example_calib.json")

if not calib.exists():
    raise ValueError(f"Calibration file not found at path {calib.absolute().as_posix()}")

with open(calib, 'r', encoding="utf-16") as f:
    calib = json.load(f)

cam_ids = []

for cam_name, cam_properties in calib.items():
    cam_id = int(cam_properties["CameraID"])
    cam_ids.append(cam_id)

# use_sync=False is crashing for now
optitrack = optitrack_thread.OptitrackThread(exposure=4000, delay_strobe=False, framerate=120, cam_ids=cam_ids, use_sync=True)
optitrack.start()
fps_history = []
last_frame_time = None

num_cams = 1
output_width  = int(640 / 2)


def make_mosaic(frames):
    """Arrange camera frames in a grid mosaic."""
    n = frames.shape[0]
    cols = int(np.ceil(np.sqrt(n)))
    rows = int(np.ceil(n / cols))
    h, w = frames.shape[1], frames.shape[2]
    # Pad with blank frames if needed
    blank = np.zeros((h, w), dtype=frames.dtype)
    padded = list(frames) + [blank] * (rows * cols - n)
    grid_rows = [np.hstack(padded[r * cols:(r + 1) * cols]) for r in range(rows)]
    return np.vstack(grid_rows)

print("Starting to retrieve frame groups...")
keypress = cv2.waitKey(1)
while not keypress & 0xFF == ord('q'):
    if optitrack.newFrame:
        image_frame = optitrack.read()
        num_cams = image_frame.shape[0]
        if image_frame.shape[1] > 1:
            # mosaic = make_mosaic(image_frame)
            # output_height, output_width = mosaic.shape[:2]

            now = time.perf_counter()
            if last_frame_time is not None:
                fps_history.append(1.0 / (now - last_frame_time))
                if len(fps_history) > 30:
                    fps_history.pop(0)
            last_frame_time = now

            fps = sum(fps_history) / len(fps_history) if fps_history else 0.0

            print(fps)

            # cv2.putText(mosaic, f"FPS: {fps:.1f}", (10, 30),
            #             cv2.FONT_HERSHEY_SIMPLEX, 1.0, 255, 2, cv2.LINE_AA)

            # cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
            # cv2.resizeWindow("Frame", 1280, 720)
            # cv2.imshow("Frame", mosaic)

    # keypress = cv2.waitKey(1)

    optitrack.stop()
    optitrack.join()

    cv2.destroyAllWindows()
    print("Fin!")
