import os
import sys

# Look for the OptiTrack Camera SDK in the system environment variables
if sys.platform == "win32":
    camera_sdk_dir = os.environ.get("NP_CAMERASDK", "")
    try:
        os.add_dll_directory(os.path.join(camera_sdk_dir, "lib"))
    except (OSError, ValueError):
        pass

from .pyopticam_ext import GetCameraList, eVideoMode, eImagerGain, eCameraState, eStatusLEDs, eOptimization, sStatusLightColor, cCameraLibraryStartupSettings, FrameGroup, Modes, GetFrameGroup, FillTensorFromFrameGroup, GetFrameGroupObjectArray, GetSlowFrameArray, GetFrameArrayNoSync, Frame, Camera, CameraManager, CameraEntry, CameraList, HardwareKeyList, HubList, HardwareDeviceList, CameraManagerListener, cUID, cModuleSync
