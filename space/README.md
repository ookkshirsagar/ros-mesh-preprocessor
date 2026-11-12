---
title: urdf-mesh-optimizer
emoji: 🦾
colorFrom: blue
colorTo: red
sdk: gradio
sdk_version: "6.31.0"
python_version: "3.12"
app_file: app.py
pinned: false
license: mit
short_description: Lightweight collision meshes for ROS 2 and MoveIt robots
tags:
  - robotics
  - ros2
  - urdf
  - moveit
  - mesh
---

# urdf-mesh-optimizer

Turn heavy CAD meshes into lightweight visual and collision meshes for ROS 2, MoveIt and Gazebo.

- **Single mesh:** upload an STL / OBJ / PLY / GLB, choose a triangle budget, compare before and after in 3D, download the result.
- **Whole robot:** upload a zipped `*_description` package; every collision mesh is reduced and the URDF is rewritten to match.

Command-line tool, docs and source: <https://github.com/ookkshirsagar/urdf-mesh-optimizer>

```bash
pipx install urdf-mesh-optimizer
```

This Space is deployed automatically from the GitHub repository.
