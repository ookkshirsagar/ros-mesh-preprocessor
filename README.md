# ros-mesh-preprocessor

> **Geometry preprocessing toolkit for ROS2, MoveIt, and industrial robot cell modelling.**

A focused Python CLI and library that takes high-polygon meshes and turns them into clean, performance-ready assets for robot simulation, motion planning, and URDF authoring — with measurable quality validation built in.

---

## Why this exists

Loading a raw CAD export into MoveIt or Gazebo with 200 000 triangles kills real-time planning. This tool solves that problem and goes further:

| Feature | What it does |
|---|---|
| **QEM Decimation** | Reduces triangle count while preserving geometric shape |
| **Visual / Collision pair** | One command → `_visual.stl` + `_collision.stl` + URDF `<geometry>` snippet |
| **Batch processing** | Process an entire robot cell folder in one command |
| **Hausdorff validation** | Quantifies shape deviation before you commit to a simplification level |
| **JSON benchmark report** | Records triangle count, file size, processing time, and error metrics |
| **Cross-platform** | Windows and Ubuntu, Python 3.10+ |

---

## Installation

```bash
# Clone
git clone https://github.com/ookkshirsagar/ros-mesh-preprocessor.git
cd ros-mesh-preprocessor

# Create virtual environment (recommended)
python3 -m venv venv

# Activate
source venv/bin/activate        # Linux / macOS
venv\Scripts\activate           # Windows PowerShell

# Install dependencies
pip install --upgrade pip
pip install -r requirements.txt
```

---

## Usage

### Single file

```bash
# Reduce to an absolute triangle count
python -m optimizer.cli --input model.stl --output model_small.obj --triangles 5000

# Reduce by percentage (keep 20% of original)
python -m optimizer.cli --input env.ply --output env_small.ply --reduction 80

# Save a JSON report alongside the output
python -m optimizer.cli --input model.stl --output model_small.stl --triangles 5000 --save-report
```

### URDF pair — visual + collision mesh

```bash
python -m optimizer.cli \
  --input robot_cell.stl \
  --urdf-output ./output \
  --visual 20000 \
  --collision 3000
```

Produces:
```
output/
├── robot_cell_visual.stl
├── robot_cell_collision.stl
├── robot_cell_urdf_snippet.xml     ← drop-in <geometry> tags
├── robot_cell_visual_report.json
└── robot_cell_collision_report.json
```

The XML snippet is ready to paste into your URDF:

```xml
<link name="robot_cell">
  <visual>
    <geometry>
      <mesh filename="package://YOUR_PACKAGE/meshes/robot_cell_visual.stl" scale="1 1 1"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <mesh filename="package://YOUR_PACKAGE/meshes/robot_cell_collision.stl" scale="1 1 1"/>
    </geometry>
  </collision>
</link>
```

### Batch — entire robot cell folder

```bash
python -m optimizer.cli \
  --folder ./cell_assets \
  --output ./optimized \
  --collision 5000 \
  --save-report
```

Processes every STL / OBJ / PLY / OFF / GLB in the folder and saves a combined `batch_report.json`.

---

## Benchmark report

Every run can emit a structured JSON report:

```json
{
  "input_file": "robot_cell.stl",
  "output_file": "output/robot_cell_collision.stl",
  "original_triangles": 81920,
  "reduced_triangles": 3000,
  "reduction_percent": 96.34,
  "file_size_before_mb": 4.096,
  "file_size_after_mb": 0.151,
  "processing_time_sec": 1.82,
  "hausdorff_max": 0.012341,
  "hausdorff_mean": 0.001102,
  "hausdorff_rms": 0.002017
}
```

**Hausdorff distance** (in mesh units) quantifies the maximum and average geometric deviation between the original and simplified surface — letting you make an informed decision about how aggressively to decimate without a visual inspection.

---

## Project structure

```
ros-mesh-preprocessor/
│
├── optimizer/
│   ├── __init__.py
│   ├── core.py           # Load, clean, simplify, save
│   ├── metrics.py        # Hausdorff distance + benchmark report
│   ├── urdf_export.py    # Visual/collision pair + URDF snippet
│   └── cli.py            # argparse CLI
│
├── examples/
│   └── moveit_cell_demo/
│       └── run_demo.py   # End-to-end demo (generates synthetic mesh if needed)
│
├── benchmarks/
│   └── performance_results.json
│
├── tests/
│   └── test_core.py
│
├── requirements.txt
├── LICENSE
└── README.md
```

---

## Running the demo

```bash
python examples/moveit_cell_demo/run_demo.py
```

This creates a synthetic high-polygon mesh and runs the full URDF export pipeline, printing a benchmark report to stdout.

---

## Running tests

```bash
pytest tests/ -v
```

---

## Supported formats

| Format | Extension | Read | Write |
|---|---|---|---|
| STL | `.stl` | ✅ | ✅ |
| OBJ | `.obj` | ✅ | ✅ |
| PLY | `.ply` | ✅ | ✅ |
| OFF | `.off` | ✅ | ✅ |
| GLB / GLTF | `.glb` `.gltf` | ✅ | ✅ |

Format conversion is free — specify a different extension on `--output`.

---

## Roadmap

- [ ] `--format` flag for explicit format override
- [ ] Voxel-based simplification mode (alternative to QEM)
- [ ] ROS2 launch file integration example
- [ ] Planning time benchmark (MoveIt before/after comparison)
- [ ] Dockerfile for CI

---

## Contributing

Pull requests are welcome. For major changes, open an issue first to discuss what you would like to change.

```bash
# Format
black optimizer/ tests/

# Lint
flake8 optimizer/ tests/

# Test
pytest tests/ -v
```

---

## License

[MIT](LICENSE)

---

## Related tools

- [Open3D](http://www.open3d.org/) — the geometry processing engine powering this toolkit
- [MoveIt](https://moveit.ros.org/) — motion planning framework for ROS2
- [Meshlab](https://www.meshlab.net/) — GUI alternative for manual mesh inspection
