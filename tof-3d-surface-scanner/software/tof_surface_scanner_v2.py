"""Live 3D surface scanner viewer (PC side), v2.

Receives 8x8 ToF frames + pitch/roll from the ESP32-S3 over WebSocket,
shows a 2D depth heatmap (OpenCV) and a 3D point cloud / mesh (Open3D).

Keys (click the Open3D window first):
  M  build mesh from the AVERAGE of the last AVG_FRAMES frames (hold the wand still)
  E  export  .stl (mesh), _mesh.ply (coloured mesh), _pose_points.ply, _sweep_points.ply
  C  clear   Q  quit

Connect your PC to the Wi-Fi network '3D_Scanner_AP' (password scanner123) first.

What changed from v1:
  * M averages many frames (median) instead of using one noisy frame
  * orientation calibration flags (see CONFIG) so you can fix mirrored / swapped axes
  * STL export now works (Open3D needs triangle normals) and PLY files are written
    for CloudCompare
  * dimensions are measured on the raw points, not on the smoothed mesh
  * optional "object only" size estimate (object vs. background border)
  * parse errors are printed instead of silently ignored

NOTE: orientation only (pitch + roll from the accelerometer). No yaw, no translation.
"""
import json
import math
import threading
import queue
import time
import sys
import warnings
from collections import deque

import numpy as np
import open3d as o3d
import websocket          # pip package name: websocket-client
import cv2
import matplotlib.pyplot as plt

# ======================= CONFIG =======================
ESP32_WS_URL = "ws://192.168.4.1:81"
GRID_SIZE = 8
FOV_DEG = 45.0                      # horizontal / vertical FoV of VL53L5CX
FOV_RAD = math.radians(FOV_DEG)

MIN_SCAN_DIST = 40.0                # mm
MAX_SCAN_DIST = 1000.0              # mm
MAX_TRIANGLE_DEPTH_DELTA = 30.0     # mm, max depth jump inside one triangle

AVG_FRAMES = 15                     # frames averaged when you press M (~1 s at 15 Hz)
MAX_BUFFER_POINTS = 20000           # live sweep cloud size
OBJECT_MIN_HEIGHT = 8.0             # mm an object must stand above the background

# ---- Orientation calibration (change these after the tilt test in the guide) ----
FLIP_X = False                      # True if the scene looks mirrored left/right
FLIP_Y = False                      # True if the scene looks upside down
SWAP_PITCH_ROLL = False             # True if tilting one way moves the other axis
PITCH_SIGN = 1.0                    # -1.0 if tilt correction goes the wrong direction
ROLL_SIGN = 1.0
TRANSPOSE_ROTATION = False          # True to use the inverse rotation
# ==================================================

# --- GLOBAL STATE ---
current_depth_grid = np.zeros((8, 8), dtype=np.float32)
current_pitch = 0.0
current_roll = 0.0

recent_frames = deque(maxlen=60)    # (grid, pitch, roll)
point_cloud_buffer = []             # accumulated world points (sweep mode)
last_generated_mesh = None
last_raw_points = None              # averaged single-pose points (N,3), mm
last_debug_print = 0.0

running = True
data_lock = threading.Lock()
frame_queue = queue.Queue(maxsize=10)
ws_app = None


# ---------------------------------------------------------------- geometry
def get_rotation_matrix(pitch_deg, roll_deg):
    """3x3 rotation from IMU pitch/roll (degrees), with calibration flags."""
    if SWAP_PITCH_ROLL:
        pitch_deg, roll_deg = roll_deg, pitch_deg
    pitch = math.radians(PITCH_SIGN * pitch_deg)
    roll = math.radians(ROLL_SIGN * roll_deg)

    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(pitch), -math.sin(pitch)],
        [0, math.sin(pitch), math.cos(pitch)],
    ])
    Ry = np.array([
        [math.cos(roll), 0, math.sin(roll)],
        [0, 1, 0],
        [-math.sin(roll), 0, math.cos(roll)],
    ])
    R = Ry @ Rx
    return R.T if TRANSPOSE_ROTATION else R


def zone_rays():
    """tan(angle) for every column (x) and row (y); zone centres span +-19.7 deg."""
    idx = np.arange(GRID_SIZE) - (GRID_SIZE - 1) / 2.0
    step = FOV_RAD / GRID_SIZE
    t = np.tan(idx * step)
    tx = -t if FLIP_X else t
    ty = -t if FLIP_Y else t
    return tx, ty


def valid_mask(grid):
    return (grid >= MIN_SCAN_DIST) & (grid <= MAX_SCAN_DIST)


def grid_to_3d_points(grid_8x8, pitch_deg, roll_deg):
    """8x8 depth grid -> (N,3) rotated points in mm, plus list of (row, col)."""
    R = get_rotation_matrix(pitch_deg, roll_deg)
    tx, ty = zone_rays()
    rows, cols = np.where(valid_mask(grid_8x8))
    if len(rows) == 0:
        return np.empty((0, 3)), []
    d = grid_8x8[rows, cols].astype(np.float64)
    local = np.stack([d * tx[cols], d * ty[rows], d], axis=1)
    world = local @ R.T
    return world, list(zip(rows.tolist(), cols.tolist()))


def average_recent_frames(n=AVG_FRAMES):
    """Median depth per zone over the last n frames. Returns None if too few frames."""
    with data_lock:
        frames = list(recent_frames)[-n:]
    if len(frames) < 3:
        return None

    grids = np.stack([f[0] for f in frames])
    valid = valid_mask(grids)
    g = np.where(valid, grids, np.nan)
    enough = valid.sum(axis=0) >= len(frames) / 2.0     # zone valid in >= half the frames

    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        med = np.nanmedian(g, axis=0)
        std = np.nanstd(g, axis=0)

    med = np.where(enough, med, 0.0)
    med = np.nan_to_num(med, nan=0.0).astype(np.float32)
    noise = float(np.nanmean(std[enough])) if enough.any() else float("nan")
    pitch = float(np.mean([f[1] for f in frames]))
    roll = float(np.mean([f[2] for f in frames]))
    return med, pitch, roll, len(frames), noise


def estimate_object_size(grid, pitch, roll):
    """Object vs. background using the border zones as the background level.
    Only meaningful when the object is smaller than the field of view and the
    wand looks roughly straight down at a flat table."""
    v = valid_mask(grid)
    border = np.concatenate([grid[0, :], grid[-1, :], grid[1:-1, 0], grid[1:-1, -1]])
    border = border[(border >= MIN_SCAN_DIST) & (border <= MAX_SCAN_DIST)]
    if len(border) < 8:
        return None
    bg = float(np.median(border))
    obj = v & (grid < bg - OBJECT_MIN_HEIGHT)
    if obj.sum() < 2:
        return None

    masked = np.where(obj, grid, 0.0).astype(np.float32)
    pts, _ = grid_to_3d_points(masked, pitch, roll)
    ext = pts.max(axis=0) - pts.min(axis=0)
    # the span between outer zone CENTRES misses half a zone on each side -> add one zone width
    zone_w = float(grid[obj].mean()) * math.tan(FOV_RAD / GRID_SIZE)
    height = bg - float(grid[obj].min())
    return bg, ext[0] + zone_w, ext[1] + zone_w, height, int(obj.sum())


def build_filtered_surface_mesh(grid_8x8, pitch_deg, roll_deg):
    """Grid triangulation (no bridging across depth jumps) + smoothing.
    Returns (mesh, raw_vertices) or (None, None)."""
    if np.sum(valid_mask(grid_8x8)) < 12:
        return None, None

    world, coords = grid_to_3d_points(grid_8x8, pitch_deg, roll_deg)
    if len(world) < 6:
        return None, None

    vertex_map = {(r, c): (i, float(grid_8x8[r, c])) for i, (r, c) in enumerate(coords)}

    triangles = []
    for r in range(GRID_SIZE - 1):
        for c in range(GRID_SIZE - 1):
            p0 = vertex_map.get((r, c))
            p1 = vertex_map.get((r, c + 1))
            p2 = vertex_map.get((r + 1, c))
            p3 = vertex_map.get((r + 1, c + 1))

            if p0 is not None and p1 is not None and p2 is not None:
                dists = [p0[1], p1[1], p2[1]]
                if max(dists) - min(dists) <= MAX_TRIANGLE_DEPTH_DELTA:
                    triangles.append([p0[0], p1[0], p2[0]])

            if p1 is not None and p3 is not None and p2 is not None:
                dists = [p1[1], p3[1], p2[1]]
                if max(dists) - min(dists) <= MAX_TRIANGLE_DEPTH_DELTA:
                    triangles.append([p1[0], p3[0], p2[0]])

    if len(triangles) == 0:
        return None, None

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(world)
    mesh.triangles = o3d.utility.Vector3iVector(np.array(triangles))

    # smooth for display (raw points are kept separately for measuring/export)
    mesh = mesh.subdivide_loop(number_of_iterations=2)
    mesh = mesh.filter_smooth_laplacian(number_of_iterations=3)
    mesh.compute_vertex_normals()

    verts = np.asarray(mesh.vertices)
    z_vals = verts[:, 2]
    norm_z = (z_vals - z_vals.min()) / (np.ptp(z_vals) + 1e-5)
    mesh.vertex_colors = o3d.utility.Vector3dVector(plt.get_cmap("jet")(norm_z)[:, :3])

    return mesh, world


# ---------------------------------------------------------------- networking
def on_message(ws, message):
    global current_depth_grid, current_pitch, current_roll, last_debug_print
    try:
        data = json.loads(message)
        pitch = float(data.get("pitch", 0.0))
        roll = float(data.get("roll", 0.0))
        grid = data.get("grid", [])
        if len(grid) != 64:
            return

        grid_arr = np.array(grid, dtype=np.float32).reshape((8, 8))

        with data_lock:
            current_pitch = pitch
            current_roll = roll
            current_depth_grid = grid_arr
            recent_frames.append((grid_arr, pitch, roll))

        if not frame_queue.full():
            frame_queue.put_nowait((grid_arr, pitch, roll))

        now = time.time()
        if now - last_debug_print > 1.0:
            last_debug_print = now
            n_valid = int(valid_mask(grid_arr).sum())
            print(f"[STREAM OK] Pitch: {pitch:.1f} | Roll: {roll:.1f} | Valid zones: {n_valid}/64")
    except Exception as e:
        print("[PARSE ERROR]", e)


def frame_processor_worker():
    global point_cloud_buffer
    while running:
        try:
            grid_array, pitch, roll = frame_queue.get(timeout=0.5)
        except queue.Empty:
            continue
        pts_3d, _ = grid_to_3d_points(grid_array, pitch, roll)
        if len(pts_3d) > 0:
            with data_lock:
                point_cloud_buffer.extend(pts_3d)
                if len(point_cloud_buffer) > MAX_BUFFER_POINTS:
                    point_cloud_buffer = point_cloud_buffer[-MAX_BUFFER_POINTS:]


def websocket_thread():
    global ws_app
    while running:
        try:
            print(f"[INFO] Connecting to ESP32 WebSocket at {ESP32_WS_URL} ...")
            ws_app = websocket.WebSocketApp(
                ESP32_WS_URL,
                on_open=lambda ws: print("[CONNECTED] WebSocket link active!"),
                on_message=on_message,
                on_error=lambda ws, e: print("[WS ERROR]:", e),
                on_close=lambda ws, c, m: print("[WS DISCONNECTED] Connection lost. Retrying..."),
            )
            ws_app.run_forever(ping_interval=0)
            time.sleep(1)
        except Exception:
            time.sleep(1)


# ---------------------------------------------------------------- 2D view
def update_2d_camera_feed():
    with data_lock:
        grid = current_depth_grid.copy()
        pitch = current_pitch
        roll = current_roll

    valid_points = grid[valid_mask(grid)]

    if len(valid_points) > 0:
        min_v, max_v = np.min(valid_points), np.max(valid_points)
        norm_grid = ((np.clip(grid, min_v, max_v) - min_v) / (max_v - min_v + 1e-5) * 255.0).astype(np.uint8)
        norm_grid = 255 - norm_grid
        norm_grid[~valid_mask(grid)] = 0
        status_str = f"Live OK ({len(valid_points)} pts | {int(min_v)}-{int(max_v)} mm)"
    else:
        norm_grid = np.zeros((8, 8), dtype=np.uint8)
        status_str = f"TARGET OUT OF RANGE ({int(MIN_SCAN_DIST)}-{int(MAX_SCAN_DIST)} mm)"

    depth_map = cv2.resize(norm_grid, (360, 360), interpolation=cv2.INTER_NEAREST)
    color_map = cv2.applyColorMap(depth_map, cv2.COLORMAP_JET)

    cv2.putText(color_map, f"Pitch: {pitch:.1f} deg", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(color_map, f"Roll:  {roll:.1f} deg", (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(color_map, status_str, (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    cv2.putText(color_map, "M: Mesh | E: Export | C: Clear | Q: Quit", (10, 340),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

    cv2.imshow("2D ToF Depth Camera View", color_map)
    cv2.waitKey(1)


# ---------------------------------------------------------------- main
def main():
    global running, last_generated_mesh

    threading.Thread(target=websocket_thread, daemon=True).start()
    threading.Thread(target=frame_processor_worker, daemon=True).start()

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="3D Surface Visualizer", width=1024, height=768)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.zeros((1, 3)))
    vis.add_geometry(pcd)

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=40.0)
    vis.add_geometry(axis)

    print("\n=================== 3D SURFACE SCANNER v2 ===================")
    print(" 1. Connect Wi-Fi to '3D_Scanner_AP' (password: scanner123).")
    print(" 2. Hold the wand ~15-25 cm above the object, pointing straight down.")
    print(f" 3. Keep it STILL for ~1 s, then press 'M' (averages {AVG_FRAMES} frames).")
    print(" 4. Press 'E' to export STL + PLY files into the current folder.")
    print(" 5. Press 'C' to clear, 'Q' to quit.")
    print("==============================================================\n")

    def capture_object_mesh(vis):
        global last_generated_mesh, last_raw_points

        avg = average_recent_frames()
        if avg is None:
            print("[ERROR] Not enough frames yet. Is the stream running?")
            return False
        grid, pitch, roll, n, noise = avg

        mesh, raw = build_filtered_surface_mesh(grid, pitch, roll)
        if mesh is None:
            print(f"[ERROR] Too few valid zones. Keep the surface {int(MIN_SCAN_DIST)}-"
                  f"{int(MAX_SCAN_DIST)} mm away (15-25 cm recommended), matte, not shiny.")
            return False

        last_generated_mesh = mesh
        last_raw_points = raw

        ext = raw.max(axis=0) - raw.min(axis=0)
        print("\n---------------- SCAN SUMMARY ----------------")
        print(f" Frames averaged     : {n}")
        print(f" Mean noise (1 sigma): {noise:.1f} mm per zone")
        print(f" Valid zones         : {len(raw)}/64")
        print(f" Scanned patch  X/Y/Z: {ext[0]:.1f} / {ext[1]:.1f} / {ext[2]:.1f} mm  (raw points)")
        obj = estimate_object_size(grid, pitch, roll)
        if obj is not None:
            bg, ox, oy, oh, k = obj
            print(f" Background depth    : {bg:.0f} mm")
            print(f" OBJECT estimate     : {ox:.0f} x {oy:.0f} mm footprint, {oh:.0f} mm tall ({k} zones)")
            print("   (footprint is quantised to about one zone = ~0.1 x distance)")
        else:
            print(" Object estimate     : n/a (needs an object smaller than the view, with"
                  " background visible around its edge)")
        print("----------------------------------------------\n")

        bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(
            o3d.utility.Vector3dVector(raw))
        bbox.color = (1.0, 0.2, 0.2)

        vis.clear_geometries()
        vis.add_geometry(mesh)
        vis.add_geometry(bbox)
        vis.add_geometry(axis)
        vis.poll_events()
        vis.update_renderer()
        print("[SUCCESS] Mesh built. Press 'E' to export.\n")
        return False

    def export_scan(vis):
        if last_generated_mesh is None:
            print("[ERROR] Press 'M' to build a mesh before exporting!")
            return False

        stamp = time.strftime("%Y%m%d_%H%M%S")
        mesh = o3d.geometry.TriangleMesh(last_generated_mesh)
        mesh.compute_triangle_normals()      # the STL writer needs these
        mesh.compute_vertex_normals()

        ok_stl = o3d.io.write_triangle_mesh(f"scan_{stamp}.stl", mesh)
        ok_ply = o3d.io.write_triangle_mesh(f"scan_{stamp}_mesh.ply", mesh)
        print(f"[EXPORT] scan_{stamp}.stl       : {'OK' if ok_stl else 'FAILED'}")
        print(f"[EXPORT] scan_{stamp}_mesh.ply  : {'OK' if ok_ply else 'FAILED'}")

        if last_raw_points is not None and len(last_raw_points) > 0:
            p = o3d.geometry.PointCloud()
            p.points = o3d.utility.Vector3dVector(last_raw_points)
            o3d.io.write_point_cloud(f"scan_{stamp}_pose_points.ply", p)
            print(f"[EXPORT] scan_{stamp}_pose_points.ply")

        with data_lock:
            sweep = np.array(point_cloud_buffer) if len(point_cloud_buffer) else None
        if sweep is not None:
            p = o3d.geometry.PointCloud()
            p.points = o3d.utility.Vector3dVector(sweep)
            o3d.io.write_point_cloud(f"scan_{stamp}_sweep_points.ply", p)
            print(f"[EXPORT] scan_{stamp}_sweep_points.ply ({len(sweep)} points)")
        print("Units are millimetres.\n")
        return False

    def clear_scan(vis):
        global last_generated_mesh, last_raw_points
        with data_lock:
            point_cloud_buffer.clear()
        last_generated_mesh = None
        last_raw_points = None
        vis.clear_geometries()
        vis.add_geometry(pcd)
        vis.add_geometry(axis)
        print("[CLEARED] Buffer reset.")
        return False

    def close_app(vis):
        global running
        running = False
        return False

    vis.register_key_callback(ord("M"), capture_object_mesh)
    vis.register_key_callback(ord("E"), export_scan)
    vis.register_key_callback(ord("C"), clear_scan)
    vis.register_key_callback(ord("Q"), close_app)

    try:
        while running:
            update_2d_camera_feed()

            with data_lock:
                if len(point_cloud_buffer) > 0 and last_generated_mesh is None:
                    pts = np.array(point_cloud_buffer)
                    pcd.points = o3d.utility.Vector3dVector(pts)
                    z_norm = (pts[:, 2] - pts[:, 2].min()) / (np.ptp(pts[:, 2]) + 1e-5)
                    pcd.colors = o3d.utility.Vector3dVector(plt.get_cmap("jet")(z_norm)[:, :3])
                    vis.update_geometry(pcd)

            if not vis.poll_events():
                break
            vis.update_renderer()
    finally:
        running = False
        if ws_app:
            ws_app.close()
        cv2.destroyAllWindows()
        vis.destroy_window()
        sys.exit(0)


if __name__ == "__main__":
    main()
