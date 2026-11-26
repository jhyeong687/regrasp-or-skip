"""Measure the jaw line against the workpiece from rendered, calibrated RGB-D."""

import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw

CAMERA = "alignment_front"
RESOLUTION = (640, 480)
MARKER_GROUP = 5
PALETTES = (("left", ".05 1 .12 1"), ("right", ".10 .30 1 1"))
GUIDANCE = """The visual_alignment packet measures the jaw line from a fixed front RGB-D view at
pinch and the target footprint from wrist depth before contact. It uses optical jaw markers.
Use the along_finger_offset_mm and target_along_finger_bounds_mm to check whether the pad
midpoint is inside the object footprint along the fingers. A midpoint outside that footprint
by several mm is geometric evidence of a missed pinch even if side silhouettes overlap.
An aligned pinch can still lose grip during lift. Compare this geometric evidence to the
camera sequence and any other supplied observations. Choose one of the existing actions.
When the alignment measurement is unavailable, use the other observations."""


def add_alignment_view(world):
    ET.SubElement(world, "camera", name=CAMERA, pos=".490 .011 .125",
                  xyaxes="0 1 0 -.212 0 .977", resolution="640 480", sensorsize=".0064 .0048",
                  focalpixel="710 710", principalpixel="0 0")
    for side, color in PALETTES:
        jaw = world.find(f".//body[@name='jaw_{side}']")
        sensor = jaw.find(f"site[@name='{side}_sensor_frame']")
        xyz = np.fromstring(sensor.attrib["pos"], sep=" ")
        # Both sensor frames share +Y as pad-up. The visual marks are
        # centered along the fingers and sit above the pad faces.
        xyz[1] += .033
        ET.SubElement(jaw, "site", name=f"{side}_alignment_mark", type="sphere",
                      pos=" ".join(map(str, xyz)), size=".004", rgba=color, group=str(MARKER_GROUP))


def camera_calibration(bench):
    camera = bench.model.camera(CAMERA).id
    intrinsics = bench.model.cam_intrinsic[camera] * np.tile(
        bench.model.cam_resolution[camera] / bench.model.cam_sensorsize[camera], 2)
    fx, fy, ox, oy = intrinsics
    cx, cy = np.asarray(RESOLUTION) / 2 + [ox, -oy]
    k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    world = np.eye(4)
    world[:3, :3] = bench.data.cam_xmat[camera].reshape(3, 3) @ np.diag([1., -1., -1.])
    world[:3, 3] = bench.data.cam_xpos[camera]
    return k, world


def unproject(depth, mask, k, world):
    rows, cols = np.nonzero(mask & np.isfinite(depth) & (depth > 0))
    z = depth[rows, cols]
    pixels = np.column_stack((cols, rows)) + .5
    xyz = np.column_stack(((pixels - k[:2, 2]) * z[:, None] / k.diagonal()[:2], z))
    return (xyz @ world[:3, :3].T + world[:3, 3]) * 1000


def wrist_footprint(sensed):
    _, depth, _, mask, k, world = sensed
    points = unproject(depth, mask, k, world)
    upper = np.percentile(points[:, 2], 92)
    return points[points[:, 2] >= upper - 2.2]


def capture_alignment(bench):
    previous = bench.option.sitegroup[MARKER_GROUP]
    bench.option.sitegroup[MARKER_GROUP] = 1
    try:
        rgb = bench._render(CAMERA, RESOLUTION)
        depth = bench._render(CAMERA, RESOLUTION, "depth")
    finally:
        bench.option.sitegroup[MARKER_GROUP] = previous
    k, world = camera_calibration(bench)
    return rgb, depth, k, world


def measure_alignment(footprint, aligned):
    rgb, depth, k, world = aligned
    r, g, b = rgb.astype(np.int16).transpose(2, 0, 1)
    masks = ((g > 120) & (g > r + 55) & (g > b + 45),
             (b > 135) & (b > r + 72) & (b > g + 42))
    markers = []
    for mask in masks:
        points = unproject(depth, mask, k, world)
        if len(points) < 12:
            return {"available": False, "reason": "jaw markers not visible"}, None
        markers.append(np.median(points, axis=0))
    left, right = np.asarray(markers)
    across = right[:2] - left[:2]
    width = np.linalg.norm(across)
    if not 12 <= width <= 82:
        return {"available": False, "reason": "implausible jaw marker separation"}, None
    across /= width
    along = np.array([across[1], -across[0]])
    if along[0] < 0:
        along *= -1
    midpoint = (left[:2] + right[:2]) / 2
    footprint_xy = footprint[:, :2]
    along_target = (footprint_xy - midpoint) @ along
    a, b = np.percentile(along_target, [2, 98])
    center = (np.percentile(footprint_xy, 2, axis=0) + np.percentile(footprint_xy, 98, axis=0)) / 2
    margin = max(a, -b, 0)
    return {
        "available": True,
        "jaw_midpoint_world_xy_mm": midpoint.round(2).tolist(),
        "target_center_world_xy_mm": center.round(2).tolist(),
        "along_finger_offset_mm": round(float((center - midpoint) @ along), 2),
        "across_jaw_offset_mm": round(float((center - midpoint) @ across), 2),
        "target_along_finger_bounds_mm": np.round([a, b], 2).tolist(),
        "jaw_midpoint_outside_footprint_mm": round(float(margin), 2),
        "jaw_marker_separation_mm": round(float(width), 2),
    }, (np.array(markers), footprint)


def alignment_image(aligned, overlay, metrics):
    rgb, _, k, world = aligned
    image = Image.fromarray(rgb)
    draw = ImageDraw.Draw(image)
    if overlay is not None:
        markers, footprint = overlay
        def project(xyz):
            optical = (xyz / 1000 - world[:3, 3]) @ world[:3, :3]
            pixel = optical @ k.T
            return [tuple(row[:2] / row[2]) for row in pixel]
        left, right = project(markers)
        draw.line((left,right), fill="#eaf7f7", width=2)
        center = metrics["target_center_world_xy_mm"]
        tx,ty = project(np.array([[*center, np.median(footprint[:, 2])]]))[0]
        draw.ellipse((tx - 5, ty - 5, tx + 5, ty + 5), outline="#ffd68a", width=2)
        draw.line((tx,ty,(left[0]+right[0])/2,(left[1]+right[1])/2),fill="#ffd68a",width=2)
    return image

def visual_packet(original, path, aligned, overlay, metrics):
    """Keep the original views and append the calibrated alignment view and plot."""
    original = Image.open(original).convert("RGB")
    width, old_height = original.size
    panel_width, height = width // 2, 216
    banner = 22
    result = Image.new("RGB", (width, old_height + height + banner), "#142032")
    result.paste(original, (0, 0))
    photo = alignment_image(aligned, overlay, metrics)
    photo.thumbnail((panel_width, height), Image.Resampling.LANCZOS)
    result.paste(photo, ((panel_width - photo.width)//2, old_height + banner))
    draw = ImageDraw.Draw(result)
    draw.text((7, old_height + 4), "PINCH | FIXED FRONT RGB-D", fill="white")
    draw.text((panel_width + 7, old_height + 4), "CALIBRATED TOP VIEW | mm", fill="white")
    if overlay is not None:
        markers, footprint = overlay
        target_xy = np.asarray(metrics["target_center_world_xy_mm"])
        midpoint = np.asarray(metrics["jaw_midpoint_world_xy_mm"])
        origin = np.array([panel_width * 1.5, old_height + banner + height / 2])
        world_center = (target_xy + midpoint) / 2
        scale = 3.2
        def pixel(point):
            return tuple(origin + (np.asarray(point)[:2] - world_center) * [scale, -scale])
        for point in footprint[::2]:
            u, v = pixel(point)
            draw.point((u, v), fill="#e4ad67")
        a, b = [pixel(point) for point in markers]
        draw.line((a,b),fill="#6ce0f4",width=3)
        for point, color in ((target_xy, "#ffc47c"), (midpoint, "#6ce0f4")):
            u,v = pixel(point)
            draw.ellipse((u-4,v-4,u+4,v+4), outline=color, width=2)
        u = metrics["jaw_midpoint_outside_footprint_mm"]
        draw.text((panel_width + 8, old_height + banner + height - 18),
                  f"Jaw midpoint beyond target footprint: {u:.2f} mm", fill="white")
    else:
        draw.text((panel_width + 9, old_height + banner + 15), metrics["reason"], fill="#ffc47c")
    result.save(path, quality=88)
