"""
Converts a race stack map (stack_master/maps/<map_name>) into the geometry that the AutoDRIVE simulator
builds at runtime (MapLoader.cs): duct walls, checkpoints, finish line and start pose.

All coordinates are in the stack's `map` frame (meters, yaw in radians). MapLoader.cs converts them to Unity.
"""

import base64
import json
import math
import os

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage
from skimage import measure

DUCT_RADIUS = 0.15  # m, radius of the round wall ducts (smaller where the wall is thin)
# m, height of the LiDAR above the ground: the rear axle is 0.059 m above the ground at rest (z of
# /autodrive/roboracer_1/ips) and the LiDAR 0.096 m above the rear axle (`lidar` frame of autodrive_bridge)
LIDAR_HEIGHT = 0.155
SMOOTHING = 0.1  # m, smoothing of the pixel staircase of the track edge
WALL_SPACING = 0.1  # m between the points of the duct centerlines
MIN_OFFSET = 0.03  # m, thinnest wall half-thickness that is still hit by the LiDAR
CHECKPOINT_SPACING = 2.0  # m along the raceline
CHECKPOINT_MARGIN = 0.2  # m added to the width of the checkpoint triggers


def build_map_message(map_path):
    """Return the map in `map_path` as a base64 encoded JSON string, ready for the `Map` field of the bridge reply."""
    return base64.b64encode(json.dumps(build_map(map_path), separators=(',', ':')).encode()).decode()


def build_map(map_path):
    """Return the simulator geometry of the map in `map_path` as a dict."""
    name = os.path.basename(os.path.normpath(map_path))
    with open(os.path.join(map_path, name + '.yaml')) as f:
        info = yaml.safe_load(f)
    with open(os.path.join(map_path, 'global_waypoints.json')) as f:
        wpnts = json.load(f)['global_traj_wpnts_iqp']['wpnts']  # the raceline the stack drives

    # the last waypoint closes the loop and repeats the first one
    if math.hypot(wpnts[-1]['x_m'] - wpnts[0]['x_m'], wpnts[-1]['y_m'] - wpnts[0]['y_m']) < 1e-3:
        wpnts = wpnts[:-1]
    raceline = {k: np.array([w[k] for w in wpnts]) for k in ('s_m', 'x_m', 'y_m', 'psi_rad', 'd_left', 'd_right')}

    walls = wall_loops(os.path.join(map_path, info['image']), info, raceline['x_m'], raceline['y_m'])
    checkpoints = checkpoint_poses(raceline)
    return {
        'name': name,
        'walls': [{'points': np.round(w.ravel(), 3).tolist()} for w in walls],  # x, y, radius of each point
        'spawn': pose(raceline['x_m'][0], raceline['y_m'][0], raceline['psi_rad'][0]),
        'finish': checkpoints[0],
        'checkpoints': checkpoints,
    }


def wall_loops(image_path, info, raceline_x, raceline_y):
    """
    Return the wall ducts as a list of closed (N, 3) polylines of (x, y, duct radius).

    The track is the free area that contains the start of the raceline. Its boundaries (outside of the track and
    infield) are the surface that the LiDAR sees. The duct centerlines are offset from them, into the walls, so that
    the duct surface at LiDAR height lies on the boundary. Where a wall is thinner than a duct, the duct shrinks to
    the thickness of the wall.
    """
    res = info['resolution']
    ox, oy = info['origin'][:2]
    img = np.array(Image.open(image_path).convert('L'), dtype=float)
    prob = img / 255 if info.get('negate', 0) else (255 - img) / 255
    occupied = prob > info['occupied_thresh']
    h, w = occupied.shape

    # pixel of each raceline point
    rows = np.round(h - 1 - ((raceline_y - oy) / res - 0.5)).astype(int)
    cols = np.round((raceline_x - ox) / res - 0.5).astype(int)
    if not (np.all((rows >= 0) & (rows < h)) and np.all((cols >= 0) & (cols < w))):
        raise ValueError(f'the raceline leaves the image {image_path}')
    if occupied[rows[0], cols[0]]:
        raise ValueError(f'the start of the raceline is inside a wall of {image_path}')
    labels, _ = ndimage.label(~occupied)  # 4-connected, so that diagonal wall pixels are closed
    track = labels == labels[rows[0], cols[0]]
    if not np.all(track[rows, cols]):
        raise ValueError(f'the raceline is not inside one free area of {image_path}')
    to_edge = ndimage.distance_transform_edt(~track) * res - 0.5 * res  # distance into the walls from the track edge

    # edge of the track, as closed polylines in world coordinates (pad: the image border is a wall for the tracing)
    outlines = []
    for c in measure.find_contours(np.pad(track, 1).astype(float), 0.5):
        if not np.allclose(c[0], c[-1]) or len(c) * res < 1.0:
            continue  # not a closed outline, or a speck
        x = ox + (c[:-1, 1] - 1 + 0.5) * res
        y = oy + (h - 1 - (c[:-1, 0] - 1) + 0.5) * res
        o = resample(np.stack([x, y], axis=1), res)
        outlines.append(ndimage.gaussian_filter1d(o, SMOOTHING / res, axis=0, mode='wrap'))  # removes the staircase
    if len(outlines) < 2:
        raise ValueError(f'expected an outside and an infield outline in {image_path}, found {len(outlines)}')

    # Offset the outlines into the walls. The outermost outline has the walls outside, all others (infield) inside.
    # The outlines are `res` apart, so that the smoothing (in points) is in meters.
    offset = math.sqrt(DUCT_RADIUS ** 2 - (DUCT_RADIUS - LIDAR_HEIGHT) ** 2)  # center to surface, at LiDAR height
    areas = [signed_area(o) for o in outlines]
    outer = int(np.argmax(np.abs(areas)))
    loops = []
    for k, o in enumerate(outlines):
        if areas[k] < 0:
            o = o[::-1]  # counter-clockwise, so that the right of the direction of travel points outwards
        t = np.roll(o, -1, axis=0) - np.roll(o, 1, axis=0)
        normal = np.stack([t[:, 1], -t[:, 0]], axis=1) / np.linalg.norm(t, axis=1, keepdims=True)
        if k != outer:
            normal = -normal  # into the infield

        # If the other side of the wall is closer than the offset, the wall is thinner than a duct: center the duct
        p = o + offset * normal
        other = ndimage.map_coordinates(to_edge, [h - 1 - ((p[:, 1] - oy) / res - 0.5), (p[:, 0] - ox) / res - 0.5],
                                        order=1, mode='nearest')
        half = np.where(other < offset - 0.01, (offset + other) / 2, offset)
        half = np.clip(ndimage.gaussian_filter1d(half, SMOOTHING / res, mode='wrap'), MIN_OFFSET, offset)
        radius = (half ** 2 + LIDAR_HEIGHT ** 2) / (2 * LIDAR_HEIGHT)  # duct whose LiDAR-height surface is `half` out
        loops.append(resample(np.column_stack([o + half[:, None] * normal, radius]), WALL_SPACING))
    return loops


def signed_area(polygon):
    """Return the area of a closed polyline, positive if it runs counter-clockwise."""
    x, y = polygon[:, 0], polygon[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def resample(polygon, spacing):
    """Return a closed polyline resampled to points `spacing` apart along its x, y; further columns are interpolated."""
    closed = np.vstack([polygon, polygon[:1]])
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(closed[:, :2], axis=0), axis=1))])
    s = np.linspace(0.0, arc[-1], max(int(round(arc[-1] / spacing)), 3), endpoint=False)
    return np.column_stack([np.interp(s, arc, column) for column in closed.T])


def checkpoint_poses(raceline):
    """
    Return the checkpoint triggers, evenly spaced along the raceline. Checkpoint 0 is at the start line and doubles
    as the finish line. Each trigger spans the track from wall to wall.
    """
    s = raceline['s_m']
    length = s[-1] + math.hypot(raceline['x_m'][0] - raceline['x_m'][-1], raceline['y_m'][0] - raceline['y_m'][-1])
    count = max(int(round(length / CHECKPOINT_SPACING)), 3)
    poses = []
    for k in range(count):
        i = int(np.argmin(np.abs(s - k * length / count)))
        yaw = raceline['psi_rad'][i]
        d_left, d_right = raceline['d_left'][i], raceline['d_right'][i]
        shift = (d_left - d_right) / 2  # centers the trigger between the walls
        poses.append(pose(raceline['x_m'][i] - math.sin(yaw) * shift, raceline['y_m'][i] + math.cos(yaw) * shift, yaw,
                          d_left + d_right + CHECKPOINT_MARGIN))
    return poses


def pose(x, y, yaw, width=None):
    p = {'x': round(float(x), 4), 'y': round(float(y), 4), 'yaw': round(float(yaw), 4)}
    if width is not None:
        p['width'] = round(float(width), 3)
    return p
