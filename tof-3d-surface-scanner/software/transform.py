"""Coordinate maths for the ToF wand.

Converts an 8x8 depth grid from the sensor into 3D points, then rotates them
into a world frame using IMU roll/pitch/yaw.

Assumptions to verify against your sensor's datasheet and real measurements:
  * The 8x8 grid spans about 45 x 45 degrees (about 63 degrees diagonal).
  * Reported distance is treated as range along each zone's ray.
"""
import numpy as np

GRID = 8
FOV_DEG = 45.0


def zone_angles(grid=GRID, fov_deg=FOV_DEG):
    """Return (az, el) arrays of shape (grid, grid) in radians, one per zone centre."""
    step = fov_deg / grid
    centres = (np.arange(grid) + 0.5) * step - fov_deg / 2.0
    az, el = np.meshgrid(np.radians(centres), np.radians(centres))
    return az, el


def depth_to_points(depth_mm):
    """depth_mm: (8, 8) array. Returns (64, 3) points in the sensor frame, in mm.
    Sensor frame: X right, Y up, Z forward."""
    az, el = zone_angles(depth_mm.shape[0])
    dx = np.tan(az)
    dy = np.tan(el)
    norm = np.sqrt(dx**2 + dy**2 + 1.0)
    rays = np.stack([dx / norm, dy / norm, 1.0 / norm], axis=-1)
    return (rays * depth_mm[..., None]).reshape(-1, 3)


def rotation_matrix(roll, pitch, yaw):
    """Rotation from sensor frame to world frame. Angles in radians (Z-Y-X order)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def to_world(points, roll, pitch, yaw):
    return points @ rotation_matrix(roll, pitch, yaw).T


if __name__ == "__main__":
    depth = np.full((GRID, GRID), 500.0)
    pts = depth_to_points(depth)
    assert pts.shape == (64, 3)
    assert np.allclose(np.linalg.norm(pts, axis=1), 500.0)   # range preserved
    assert np.allclose(to_world(pts, 0, 0, 0), pts)           # zero rotation = unchanged
    r = rotation_matrix(0, 0, np.pi / 2)
    assert np.allclose(r @ np.array([1, 0, 0]), [0, 1, 0])    # 90 deg yaw: X -> Y
    print("transform.py self-test passed.")
