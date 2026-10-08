# Software (PC side, Python)

| File | Purpose |
|---|---|
| `tof_surface_scanner.py` | **Main app.** Connects to the scanner over Wi-Fi (WebSocket), shows a 2D depth heatmap and a live 3D point cloud, builds a mesh (`M`), measures bounding-box size and exports STL (`E`). |
| `serial_pointcloud_viewer.py` | Stage 1 viewer over USB serial (matplotlib). Kept to show the project's progress. |

```bash
pip install -r ../requirements.txt
python tof_surface_scanner.py
```

Use a Python version that your installed Open3D release supports (check the Open3D docs if `pip install open3d` fails).
