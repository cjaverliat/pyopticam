import rerun as rr
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from pyopticam import read_mcal

def _w2c_to_c2w_pose(Rt: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Decomposes world-to-camera Rt (3,4) into c2w translation and xyzw quaternion."""
    # Extract R and t from the [R | t] matrix
    R_w2c = Rt[:3, :3]
    t_w2c = Rt[:3, 3]

    # The inverse of a rigid transform [R | t] is [R^T | -R^T @ t]
    R_c2w = R_w2c.T
    t_c2w = -R_c2w @ t_w2c

    # Convert rotation matrix to xyzw quaternion
    quat_xyzw = R.from_matrix(matrix=R_c2w).as_quat()

    return t_c2w, quat_xyzw

def log_camera(
        entity_path: str,
        K: np.ndarry,
        Rt: np.ndarry,
        width: int,
        height: int,
        image_plane_distance: float = 0.15,
        static: bool = False,
) -> None:
    """
    Logs a pinhole camera to Rerun.

    Args:
        entity_path: Rerun entity path.
        K: (3, 3) intrinsic matrix.
        Rt: (3, 4) world-to-camera extrinsic [R | t].
        width: image width in pixels.
        height: image height in pixels.
        D: distortion coefficients (unused by Rerun's Pinhole, kept for API completeness).
        image_plane_distance: distance of the image plane from the camera origin for visualization.
        static: log as a static entity.
    """
    t_c2w, quat_xyzw = _w2c_to_c2w_pose(Rt)
    rr.log(
        entity_path,
        rr.Transform3D(
            translation=t_c2w,
            rotation=rr.Quaternion(xyzw=quat_xyzw),
        ),
        rr.Pinhole(
            image_from_camera=K,
            width=width,
            height=height,
            image_plane_distance=image_plane_distance
        ),
        static=static,
    )


if __name__ == "__main__":

    rr.init("Optitrack", spawn=True)

    calib_path = Path("./assets/example_calib.mcal")
    calib = read_mcal(calib_path)

    rr.set_time("frame_idx", sequence=0)
    for cam_serial in calib.keys():
        cam_calib = calib[cam_serial]
        cam_id = cam_calib.camera_id
        Rt_i = cam_calib.extrinsic
        K_i = cam_calib.camera_matrix
        D_i = cam_calib.dist_coeffs
        log_camera(f"cam_{cam_id}", K_i, Rt_i, width=cam_calib.width, height=cam_calib.height)
