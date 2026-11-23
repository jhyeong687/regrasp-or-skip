"""Render diagnostic-only camera and tactile dashboard frames."""

from pathlib import Path

import mujoco as mj
import numpy as np
from PIL import Image

from flush_bench import COMPRESSION_POSITIVE, SETUP, VIEWS


_CAMERA_SIZE = (320, 180)
_DEPTH_COLORS = np.array([[250, 191, 101], [225, 91, 122], [118, 83, 179],
                          [38, 102, 158], [46, 154, 168]])


def photo(image, path):
    tmp = path.with_suffix('.tmp')
    image.save(tmp, format='JPEG', quality=88)
    tmp.replace(path)


def write_frame(bench, sample, wide_path: Path, cameras_path: Path, gripper_path: Path):
    wide = bench._render('bench_view', (672, 378))
    images = [bench._render(camera, _CAMERA_SIZE) for camera in VIEWS[:3]]
    depth = bench._render(VIEWS[3], _CAMERA_SIZE, 'depth')
    lower, upper = SETUP['depth_min_m'], SETUP['depth_max_m']
    valid = np.isfinite(depth) & (depth >= lower) & (depth <= upper)
    fraction = np.clip((np.where(valid, depth, lower) - lower) / (.62 - lower), 0, 1)
    stops = np.linspace(0, 1, len(_DEPTH_COLORS))
    color = np.stack([np.interp(fraction, stops, channel) for channel in _DEPTH_COLORS.T], axis=-1)
    color[~valid] = (21, 25, 36)
    segmentation = bench._render(VIEWS[3], _CAMERA_SIZE, 'segmentation')
    handle_mesh = bench.model.mesh("grip_finish").id
    handle_geoms = np.flatnonzero(
        (bench.model.geom_type == mj.mjtGeom.mjGEOM_MESH) & (bench.model.geom_dataid == handle_mesh)
    )
    target = np.isin(segmentation[..., 0], handle_geoms) & (segmentation[..., 1] == mj.mjtObj.mjOBJ_GEOM)
    measured = valid & target
    valid_target = depth[measured]
    raw_target = depth[target & np.isfinite(depth)]
    _, patch, _ = bench._sample_tactile()
    photo(Image.fromarray(wide), wide_path)
    photo(Image.fromarray(bench._render('gripper_check', (672, 378))), gripper_path)
    photo(Image.fromarray(np.hstack([*images, color.astype('uint8')])), cameras_path)
    return {
        'phase': sample['phase'],
        'sim_s': sample['t'],
        'pads': sample['pads'],
        'patch_pads': (patch * COMPRESSION_POSITIVE).round(4).tolist(),
        'gap_mm': sample['gap_mm'],
        'elbow_Nm': float(abs(bench.data.actuator_force[2])),
        'depth': {
            'valid_pct': 100 * float(valid.mean()),
            'near_pct': 100 * float((np.isfinite(depth) & (depth < lower)).mean()),
            'min_m': lower,
            'max_m': upper,
            'target_pixels': int(target.sum()),
            'target_valid': int(measured.sum()),
            'target_median_mm': float(1000 * np.median(valid_target)) if valid_target.size else None,
            'target_raw_median_mm': float(1000 * np.median(raw_target)) if raw_target.size else None,
        },
    }
