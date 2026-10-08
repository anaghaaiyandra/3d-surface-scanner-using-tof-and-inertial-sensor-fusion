"""Stage 1: USB-serial point cloud viewer (matplotlib).

Reads '8x8:' frames printed by the early serial firmware, shows them in the terminal
and accumulates a 3D point cloud. Press 'S' in the plot window to save a CSV.
Kept for reference: the Wi-Fi version (tof_surface_scanner.py) replaced it.
"""
import serial
import numpy as np
import matplotlib.pyplot as plt
import math
from datetime import datetime

# --- CONFIGURATION ---
SERIAL_PORT = 'COM4'
BAUD_RATE = 115200
GRID_SIZE = 8

# FoV calculation (VL53L5CX has a 63° diagonal FoV, approx 45° H/V)
FOV_RAD = math.radians(45.0)

# Global array to store accumulated points across all frames
accumulated_points = np.empty((0, 3))


def calculate_3d_points(depth_matrix):
    """Converts the 8x8 depth map to physical X, Y, Z coordinates in millimeters."""
    points = []
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            z = depth_matrix[y][x]

            if z > 0:
                cx = x - 3.5
                cy = y - 3.5

                px = cx * z * math.tan(FOV_RAD / 2.0) / 3.5
                py = cy * z * math.tan(FOV_RAD / 2.0) / 3.5

                points.append([px, py, z])

    return np.array(points)


def save_point_cloud(event):
    """Saves the accumulated points to a CSV file when 's' is pressed."""
    global accumulated_points
    if event.key == 's' or event.key == 'S':
        if len(accumulated_points) == 0:
            print("No points to save yet!")
            return

        filename = f"scan_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        np.savetxt(filename, accumulated_points, delimiter=",", header="X,Y,Z", comments="")
        print(f"\n--- SUCCESS ---")
        print(f"Saved {len(accumulated_points)} points to {filename}\n")


def main():
    global accumulated_points
    print(f"Connecting to ESP32 on {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    plt.ion()
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d')
    fig.canvas.mpl_connect('key_press_event', save_point_cloud)

    ax.set_title("ESP32 ToF 3D Point Cloud\n(Keep window focused and press 'S' to Save)")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")

    scatter = ax.scatter([], [], [], s=10, c='tab:blue', alpha=0.6)
    print("Listening for data... Move the scanner slowly to build the map.")

    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                if line == "8x8:":
                    depth_matrix = []
                    for _ in range(GRID_SIZE):
                        row_data = ser.readline().decode('utf-8', errors='ignore').strip()
                        values = [int(v) for v in row_data.split() if v.isdigit()]
                        if len(values) >= GRID_SIZE:
                            depth_matrix.append(values[:GRID_SIZE])

                    if len(depth_matrix) == GRID_SIZE:
                        print("\n--- New 8x8 Frame ---")
                        for row in depth_matrix:
                            print("\t".join([f"{v:4}" for v in row]))
                        print("---------------------")

                        new_points = calculate_3d_points(depth_matrix)

                        if len(new_points) > 0:
                            accumulated_points = np.vstack((accumulated_points, new_points))

                            scatter._offsets3d = (
                                accumulated_points[:, 0],
                                accumulated_points[:, 1],
                                accumulated_points[:, 2]
                            )

                            ax.set_xlim(accumulated_points[:, 0].min() - 50, accumulated_points[:, 0].max() + 50)
                            ax.set_ylim(accumulated_points[:, 1].min() - 50, accumulated_points[:, 1].max() + 50)
                            ax.set_zlim(0, accumulated_points[:, 2].max() + 100)

                            fig.canvas.draw_idle()
                            fig.canvas.flush_events()

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Closing scanner cleanly...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        plt.close('all')
        print("Shutdown complete.")


if __name__ == "__main__":
    main()
