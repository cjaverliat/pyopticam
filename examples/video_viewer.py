import cv2
import numpy as np
import time
import mp4_thread
import optitrack_thread

# A simple example for viewing and recording mp4's from your optitrack cameras
# Press 'w' to toggle recording, press `q` to quit

# use_sync=False is crashing for now
camera_serials=[107393, 107400, 107401]
optitrack = optitrack_thread.OptitrackThread(exposure=4000, delay_strobe=False, framerate=120, camera_serials=camera_serials, use_sync=True)
optitrack.start()
ffmpeg_recording = False
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


try:
    print("Starting to retrieve frame groups...")
    keypress = cv2.waitKey(1)
    while(not (keypress & 0xFF == ord('q'))):
        if optitrack.newFrame:
            image_frame = optitrack.read()
            num_cams = image_frame.shape[0]
            if image_frame.shape[1] > 1:
                mosaic = make_mosaic(image_frame)
                output_height, output_width = mosaic.shape[:2]

                if (keypress & 0xFF == ord('w')):
                    if not ffmpeg_recording:
                        ffmpeg_process = mp4_thread.ffmpegThread("OptitrackOutput.mp4", width=output_width, height=output_height)
                        ffmpeg_process.start()
                        ffmpeg_recording = True
                    else:
                        print("Killing FFMPEG process...")
                        ffmpeg_process.end_encoding()
                        ffmpeg_recording = False

                if ffmpeg_recording:
                    ffmpeg_process.add_image(mosaic)

                now = time.perf_counter()
                if last_frame_time is not None:
                    fps_history.append(1.0 / (now - last_frame_time))
                    if len(fps_history) > 30:
                        fps_history.pop(0)
                last_frame_time = now

                fps = sum(fps_history) / len(fps_history) if fps_history else 0.0
                cv2.putText(mosaic, f"FPS: {fps:.1f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, 255, 2, cv2.LINE_AA)

                cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("Frame", 1280, 720)
                cv2.imshow("Frame", mosaic)

        keypress = cv2.waitKey(1)
finally:
    if ffmpeg_recording:
        print("Killing FFMPEG process...")
        ffmpeg_process.end_encoding()
        ffmpeg_recording = False

    optitrack.stop()
    optitrack.join()

    cv2.destroyAllWindows()
    print("Fin!")
