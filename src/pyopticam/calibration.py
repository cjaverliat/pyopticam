from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
import numpy as np


@dataclass
class CameraCalibration:
    serial: int
    camera_id: int
    width: int
    height: int
    camera_matrix: np.ndarray  # shape (3, 3)
    dist_coeffs: np.ndarray    # shape (5,): [k1, k2, p1, p2, k3]
    extrinsic: np.ndarray      # shape (4, 4), OpenCV convention (+z forward)


def read_mcal(path: str | Path) -> dict[int, CameraCalibration]:
    """Parse an OptiTrack Motive .mcal file and return per-camera calibration.

    Args:
        path: Path to the .mcal file (UTF-16LE encoded XML).

    Returns:
        Dict mapping camera serial to CameraCalibration.
    """
    path = Path(path)

    if not path.exists():
        raise FileNotFoundError(f"File {path} not found.")

    tree = ET.parse(path)
    root = tree.getroot()

    cameras_el = root.find("Calibration/Cameras")
    if cameras_el is None:
        raise ValueError(f"No Calibration/Cameras element found in {path}")

    result: dict[int, CameraCalibration] = {}

    for cam_el in cameras_el.findall("Camera"):
        serial = int(cam_el.get("Serial", "0").lstrip("ABCDEFGHIJKLMNOPQRSTUVWXYZ"))

        props = cam_el.find("Properties")
        camera_id = int(props.get("CameraID", 0)) if props is not None else 0

        attrs = cam_el.find("Attributes")
        width = int(attrs.get("ImagerPixelWidth", 0)) if attrs is not None else 0
        height = int(attrs.get("ImagerPixelHeight", 0)) if attrs is not None else 0

        intr = cam_el.find("IntrinsicStandardCameraModel")
        if intr is None:
            intr = cam_el.find("Intrinsic")
        if intr is not None:
            fx = float(intr.get("HorizontalFocalLength", 0))
            fy = float(intr.get("VerticalFocalLength", 0))
            cx = float(intr.get("LensCenterX", 0))
            cy = float(intr.get("LensCenterY", 0))
            k1 = float(intr.get("k1", 0))
            k2 = float(intr.get("k2", 0))
            k3 = float(intr.get("k3", 0))
            p1 = float(intr.get("TangentialX", 0))
            p2 = float(intr.get("TangentialY", 0))
        else:
            fx = fy = cx = cy = k1 = k2 = k3 = p1 = p2 = 0.0

        camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
        dist_coeffs = np.array([k1, k2, p1, p2, k3])

        extr = cam_el.find("Extrinsic")
        if extr is not None:
            t_world = np.array([
                float(extr.get("X", 0)),
                float(extr.get("Y", 0)),
                float(extr.get("Z", 0)),
            ])
            # OrientMatrix is camera-to-world rotation (camera pose), row-major
            R_c2w = np.array(
                [float(extr.get(f"OrientMatrix{i}", 0)) for i in range(9)]
            ).reshape(3, 3)
        else:
            t_world = np.zeros(3)
            R_c2w = np.eye(3)

        R_w2c = R_c2w.T
        t_w2c = -R_w2c @ t_world

        # Convert camera frame: Motive (-Z forward, Y-up) → OpenCV (+Z forward, Y-down)
        flip = np.diag([1.0, -1.0, -1.0])
        extrinsic = np.eye(4)
        extrinsic[:3, :3] = flip @ R_w2c
        extrinsic[:3, 3] = flip @ t_w2c

        result[serial] = CameraCalibration(
            serial=serial,
            camera_id=camera_id,
            width=width,
            height=height,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            extrinsic=extrinsic,
        )

    return result
