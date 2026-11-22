"""Estimate a depth target from a caller-provided mask in the depth image."""
import numpy as np


def depth_calibration(sim):
    """Return pinhole K and world-from-optical transform (right, down, forward)."""
    camera = sim.model.camera("wrist_depth").id
    resolution = sim.model.cam_resolution[camera].astype(float)
    sensor_size = sim.model.cam_sensorsize[camera]
    scale = resolution / sensor_size
    fx, fy, cx_offset, cy_offset = sim.model.cam_intrinsic[camera] * np.tile(scale, 2)
    cx, cy = resolution / 2 + np.array([cx_offset, -cy_offset])
    k = np.array([[fx, 0., cx], [0., fy, cy], [0., 0., 1.0]])
    base_from_optical = np.eye(4)
    base_from_optical[:3, :3] = sim.data.cam_xmat[camera].reshape(3, 3) @ np.diag([1., -1., -1.])
    base_from_optical[:3, 3] = sim.data.cam_xpos[camera]
    return k, base_from_optical


def observe_target_depth(sim, depth_m, depth_mask):
    """Return depth support and a robust target-surface point in sim base axes.

    depth_mask must come from the same frame as depth_m. To use a wrist RGB
    segmentation, align the depth stream to that camera before computing its
    ROI statistics with the corresponding calibration.
    """
    values = np.asarray(depth_m)
    selected = np.asarray(depth_mask, dtype=bool)
    if values.ndim != 2 or values.shape != selected.shape:
        raise ValueError("Expected a depth image and a same-size target mask.")
    camera = sim.model.camera("wrist_depth").id
    width, height = sim.model.cam_resolution[camera]
    if values.shape != (height, width):
        raise ValueError("Expected native wrist-depth image coordinates.")
    k, transform = depth_calibration(sim)
    row, col = np.nonzero(selected & np.isfinite(values) & (values > 0))
    visible = int(selected.sum())
    result = {
        "target_pixels": visible,
        "valid_depth_pixels": int(len(row)),
        "valid_fraction": float(len(row) / visible) if visible else 0.0,
        "surface_point_base_m": None,
        "depth_percentiles_m": None,
        "intrinsics": k.tolist(),
        "base_from_optical": transform.tolist(),
    }
    if len(row):
        z = values[row, col]
        xyz = np.column_stack(((col + .5 - k[0, 2]) * z / k[0, 0],
                               (row + .5 - k[1, 2]) * z / k[1, 1], z))
        point = np.median(xyz @ transform[:3, :3].T + transform[:3, 3], axis=0)
        result.update(surface_point_base_m=point.tolist(),
                      depth_percentiles_m=np.percentile(z, [10, 50, 90]).tolist())
    return result
