"""Live 3D surface scanner viewer (PC side).

Receives 8x8 ToF frames + pitch/roll from the ESP32-S3 over WebSocket,
shows a 2D depth heatmap (OpenCV) and a 3D point cloud / mesh (Open3D).

Keys (focus the Open3D window):  M build mesh | E export STL | C clear | Q quit
Connect your PC to the Wi-Fi network '3D_Scanner_AP' first.

NOTE: orientation only (pitch + roll from the accelerometer). No yaw, no translation.
"""
import json
import math
import threading
import queue
import time
import sys
import numpy as np
import open3d as o3d
import websocket
import cv2
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
ESP32_WS_URL = "ws://192.168.4.1:81"
GRID_SIZE = 8
FOV_DEG = 45.0  # Horizontal/Vertical FOV of VL53L5CX (deg)
FOV_RAD = math.radians(FOV_DEG)

# Ranging constraints (in mm)
MIN_SCAN_DIST = 40.0
MAX_SCAN_DIST = 1000.0
MAX_TRIANGLE_DEPTH_DELTA = 30.0  # Max distance jump (mm) allowed between adjacent mesh vertices

# --- GLOBAL BUFFERS ---
current_depth_grid = np.zeros((8, 8), dtype=np.float32)
current_pitch = 0.0
current_roll = 0.0

point_cloud_buffer = []  # Stores accumulated 3D world points
last_generated_mesh = None
last_debug_print = 0

running = True
data_lock = threading.Lock()
frame_queue = queue.Queue(maxsize=10)
ws_app = None


def get_rotation_matrix(pitch_deg, roll_deg):
    """Calculates 3D rotation matrix from IMU Pitch and Roll angles."""
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)

    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(pitch), -math.sin(pitch)],
        [0, math.sin(pitch), math.cos(pitch)]
    ])

    Ry = np.array([
        [math.cos(roll), 0, math.sin(roll)],
        [0, 1, 0],
        [-math.sin(roll), 0, math.cos(roll)]
    ])

    return Ry @ Rx


def grid_to_3d_points(grid_8x8, pitch_deg, roll_deg):
    """Converts an 8x8 depth array into rotated 3D World Cartesian coordinates."""
    R = get_rotation_matrix(pitch_deg, roll_deg)
    points = []
    grid_coords = []  # Tracks (row, col) indices for topology

    half_fov = FOV_RAD / 2.0

    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            dist = grid_8x8[r, c]
            if dist < MIN_SCAN_DIST or dist > MAX_SCAN_DIST:
                continue

            # Calculate individual zone ray angles
            angle_x = (c - 3.5) * (FOV_RAD / GRID_SIZE)
            angle_y = (r - 3.5) * (FOV_RAD / GRID_SIZE)

            # Local coordinates (Z-forward)
            px = dist * math.tan(angle_x)
            py = dist * math.tan(angle_y)
            pz = dist

            local_pt = np.array([px, py, pz])
            world_pt = R @ local_pt

            points.append(world_pt)
            grid_coords.append((r, c, dist))

    return np.array(points), grid_coords


def build_filtered_surface_mesh(grid_8x8, pitch_deg, roll_deg):
    """Builds a smooth continuous 3D surface mesh with outlier edge suppression."""
    valid_mask = (grid_8x8 >= MIN_SCAN_DIST) & (grid_8x8 <= MAX_SCAN_DIST)
    if np.sum(valid_mask) < 12:
        return None, None

    R = get_rotation_matrix(pitch_deg, roll_deg)
    vertex_map = {}
    vertices = []
    vertex_count = 0

    # 1. Generate Vertices
    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            dist = grid_8x8[r, c]
            if dist < MIN_SCAN_DIST or dist > MAX_SCAN_DIST:
                continue

            angle_x = (c - 3.5) * (FOV_RAD / GRID_SIZE)
            angle_y = (r - 3.5) * (FOV_RAD / GRID_SIZE)

            px = dist * math.tan(angle_x)
            py = dist * math.tan(angle_y)
            pz = dist

            world_pt = R @ np.array([px, py, pz])
            vertices.append(world_pt)
            vertex_map[(r, c)] = (vertex_count, dist)
            vertex_count += 1

    if len(vertices) < 6:
        return None, None

    # 2. Build Triangles without bridging depth jumps
    triangles = []
    for r in range(GRID_SIZE - 1):
        for c in range(GRID_SIZE - 1):
            p0 = vertex_map.get((r, c))
            p1 = vertex_map.get((r, c + 1))
            p2 = vertex_map.get((r + 1, c))
            p3 = vertex_map.get((r + 1, c + 1))

            # Quad 1: (p0, p1, p2)
            if p0 and p1 and p2:
                dists = [p0[1], p1[1], p2[1]]
                if max(dists) - min(dists) <= MAX_TRIANGLE_DEPTH_DELTA:
                    triangles.append([p0[0], p1[0], p2[0]])

            # Quad 2: (p1, p3, p2)
            if p1 and p3 and p2:
                dists = [p1[1], p3[1], p2[1]]
                if max(dists) - min(dists) <= MAX_TRIANGLE_DEPTH_DELTA:
                    triangles.append([p1[0], p3[0], p2[0]])

    if len(triangles) == 0:
        return None, None

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(np.array(vertices))
    mesh.triangles = o3d.utility.Vector3iVector(np.array(triangles))

    # 3. Apply Smooth Shading & Depth Heatmap
    mesh = mesh.subdivide_loop(number_of_iterations=2)
    mesh.filter_smooth_laplacian(number_of_iterations=3)
    mesh.compute_vertex_normals()

    # Color Mapping (Jet Heatmap based on Z height)
    verts = np.asarray(mesh.vertices)
    z_vals = verts[:, 2]
    z_min, z_max = np.min(z_vals), np.max(z_vals)
    norm_z = (z_vals - z_min) / (z_max - z_min + 1e-5)

    cmap = plt.get_cmap("jet")
    colors = cmap(norm_z)[:, :3]
    mesh.vertex_colors = o3d.utility.Vector3dVector(colors)

    return mesh, np.array(vertices)


def on_message(ws, message):
    global current_depth_grid, current_pitch, current_roll, last_debug_print

    try:
        data = json.loads(message)
        pitch = data.get("pitch", 0.0)
        roll = data.get("roll", 0.0)
        grid = data.get("grid", [])

        if len(grid) != 64:
            return

        grid_arr = np.array(grid, dtype=np.float32).reshape((8, 8))

        with data_lock:
            current_pitch = pitch
            current_roll = roll
            current_depth_grid = grid_arr

        if not frame_queue.full():
            frame_queue.put_nowait((grid_arr, pitch, roll))

        now = time.time()
        if now - last_debug_print > 1.0:
            last_debug_print = now
            valid_pts = grid_arr[(grid_arr >= MIN_SCAN_DIST) & (grid_arr <= MAX_SCAN_DIST)]
            print(f"[STREAM OK] Pitch: {pitch:.1f}° | Roll: {roll:.1f}° | Valid depth points: {len(valid_pts)}/64")

    except Exception as e:
        pass


def frame_processor_worker():
    global point_cloud_buffer

    while running:
        try:
            grid_array, pitch, roll = frame_queue.get(timeout=0.5)
            pts_3d, _ = grid_to_3d_points(grid_array, pitch, roll)

            if len(pts_3d) > 0:
                with data_lock:
                    point_cloud_buffer.extend(pts_3d)
                    # Keep maximum 1500 points in memory
                    if len(point_cloud_buffer) > 1500:
                        point_cloud_buffer = point_cloud_buffer[-1500:]

        except queue.Empty:
            continue


def websocket_thread():
    global ws_app
    while running:
        try:
            print("[INFO] Connecting to ESP32 WebSocket at ws://192.168.4.1:81...")
            ws_app = websocket.WebSocketApp(
                ESP32_WS_URL,
                on_open=lambda ws: print("[CONNECTED] WebSocket link active!"),
                on_message=on_message,
                on_error=lambda ws, e: print("[WS ERROR]:", e),
                on_close=lambda ws, c, m: print("[WS DISCONNECTED] Connection lost. Retrying...")
            )
            ws_app.run_forever(ping_interval=0)
            time.sleep(1)
        except Exception:
            time.sleep(1)


def update_2d_camera_feed():
    with data_lock:
        grid = current_depth_grid.copy()
        pitch = current_pitch
        roll = current_roll

    valid_points = grid[(grid >= MIN_SCAN_DIST) & (grid <= MAX_SCAN_DIST)]

    if len(valid_points) > 0:
        min_v, max_v = np.min(valid_points), np.max(valid_points)
        norm_grid = ((np.clip(grid, min_v, max_v) - min_v) / (max_v - min_v + 1e-5) * 255.0).astype(np.uint8)
        norm_grid = 255 - norm_grid
        status_str = f"Live Stream OK ({len(valid_points)} pts | {int(min_v)}-{int(max_v)}mm)"
    else:
        norm_grid = np.zeros((8, 8), dtype=np.uint8)
        status_str = "TARGET OUT OF RANGE (30-1000mm)"

    depth_map = cv2.resize(norm_grid, (360, 360), interpolation=cv2.INTER_NEAREST)
    color_map = cv2.applyColorMap(depth_map, cv2.COLORMAP_JET)

    cv2.putText(color_map, f"Pitch: {pitch:.1f} deg", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(color_map, f"Roll:  {roll:.1f} deg", (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(color_map, status_str, (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    cv2.putText(color_map, "M: Build Surface | E: Export STL | C: Clear", (10, 340), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

    cv2.imshow("2D ToF Depth Camera View", color_map)
    cv2.waitKey(1)


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

    print("\n=================== 3D PRECISION SCANNER ===================")
    print(" 1. Connect Wi-Fi to '3D_Scanner_AP' (password: scanner123).")
    print(" 2. Place object on table and hold scanner ~15-25 cm directly above it.")
    print(" 3. Press 'M' : BUILD SMOOTH SURFACE MESH & HEATMAP")
    print(" 4. Press 'E' : EXPORT MESH TO FILE (.STL)")
    print(" 5. Press 'C' : CLEAR SCREEN & POINT CLOUD BUFFER")
    print(" 6. Press 'Q' : EXIT")
    print("=============================================================\n")

    def capture_object_mesh(vis):
        global last_generated_mesh

        with data_lock:
            grid = current_depth_grid.copy()
            pitch = current_pitch
            roll = current_roll

        mesh, pts_3d = build_filtered_surface_mesh(grid, pitch, roll)

        if mesh is None:
            print("[ERROR] Point cloud too sparse or out of distance bounds (15-25 cm recommended).")
            return False

        last_generated_mesh = mesh

        bbox = mesh.get_axis_aligned_bounding_box()
        bbox.color = (1.0, 0.2, 0.2)
        extent = bbox.get_extent()

        print("\n---------------- MEASURED DIMENSIONS ----------------")
        print(f" Width  (X axis) : {extent[0]:.1f} mm")
        print(f" Height (Y axis) : {extent[1]:.1f} mm")
        print(f" Depth  (Z axis) : {extent[2]:.1f} mm")
        print("-----------------------------------------------------\n")

        vis.clear_geometries()
        vis.add_geometry(mesh)
        vis.add_geometry(bbox)
        vis.add_geometry(axis)

        vis.poll_events()
        vis.update_renderer()

        print("[SUCCESS] Continuous surface mesh generated cleanly!\n")
        return False

    def export_stl_mesh(vis):
        global last_generated_mesh
        if last_generated_mesh is None:
            print("[ERROR] Press 'M' to build a mesh before exporting!")
            return False

        filename = f"scan_model_{int(time.time())}.stl"
        if o3d.io.write_triangle_mesh(filename, last_generated_mesh):
            print(f"[EXPORT SUCCESS] Saved 3D model: {filename}")
        return False

    def clear_scan(vis):
        global last_generated_mesh, point_cloud_buffer
        with data_lock:
            point_cloud_buffer.clear()
        last_generated_mesh = None
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
    vis.register_key_callback(ord("E"), export_stl_mesh)
    vis.register_key_callback(ord("C"), clear_scan)
    vis.register_key_callback(ord("Q"), close_app)

    try:
        while running:
            update_2d_camera_feed()

            with data_lock:
                if len(point_cloud_buffer) > 0 and last_generated_mesh is None:
                    pts = np.array(point_cloud_buffer)
                    pcd.points = o3d.utility.Vector3dVector(pts)

                    # Apply live point cloud color map
                    z_vals = pts[:, 2]
                    z_norm = (z_vals - np.min(z_vals)) / (np.ptp(z_vals) + 1e-5)
                    colors = plt.get_cmap("jet")(z_norm)[:, :3]
                    pcd.colors = o3d.utility.Vector3dVector(colors)

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
