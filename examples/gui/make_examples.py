"""
examples/gui/make_examples.py - Regenerate the GUI example files in this folder.

    spur_gear.stl, weld_neck_flange.stl   single-mesh examples
    demo_cell_description.zip             a small ROS description package for the
                                          "Whole robot (URDF)" tab: three links whose
                                          visual *and* collision geometry is a dense mesh

The parts come from docs/figures/parts.py, tessellated more coarsely than the README
figures so the files stay small enough for a browser upload.

Requires the 'viz' extra:  pip install -e ".[viz]"
Usage:                     python examples/gui/make_examples.py
"""

import shutil
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent / "docs"))

import trimesh  # noqa: E402
from figures import parts  # noqa: E402

COARSEN = 2.4  # multiply each part's max edge length: fewer triangles, smaller files

_original_finish = parts._finish


def _coarse_finish(mesh, max_edge_mm):
    return _original_finish(mesh, max_edge_mm * COARSEN)


parts._finish = _coarse_finish

URDF = """<?xml version="1.0"?>
<robot name="demo_cell">
  <!-- Demo cell for urdf-mesh-optimizer: every link uses its dense CAD mesh for both
       visual and collision geometry, as raw CAD-exported descriptions often do. -->
  <link name="world"/>
{links}
</robot>
"""
LINK = """  <link name="{name}">
    <visual>
      <origin xyz="{xyz}" rpy="0 0 0"/>
      <geometry><mesh filename="package://demo_cell_description/meshes/{mesh}.stl"/></geometry>
    </visual>
    <collision>
      <origin xyz="{xyz}" rpy="0 0 0"/>
      <geometry><mesh filename="package://demo_cell_description/meshes/{mesh}.stl"/></geometry>
    </collision>
  </link>
  <joint name="world_to_{name}" type="fixed">
    <parent link="world"/>
    <child link="{name}"/>
  </joint>"""


def main() -> None:
    for key in ("spur_gear", "weld_neck_flange"):
        mesh = parts.CATALOG[key][2]()
        mesh.export(HERE / f"{key}.stl")
        print(f"{key}.stl: {len(mesh.faces):,} triangles")

    with tempfile.TemporaryDirectory() as tmp:
        pkg = Path(tmp) / "demo_cell_description"
        (pkg / "meshes").mkdir(parents=True)
        (pkg / "urdf").mkdir()
        (pkg / "package.xml").write_text(
            '<?xml version="1.0"?>\n<package format="3">\n'
            "  <name>demo_cell_description</name>\n  <version>0.1.0</version>\n"
            "  <description>Demo cell for urdf-mesh-optimizer</description>\n"
            '  <maintainer email="noreply@example.com">urdf-mesh-optimizer</maintainer>\n'
            "  <license>MIT</license>\n</package>\n"
        )
        links = []
        layout = {
            "servo_motor": "0 0 0.1",
            "pillow_block": "0.3 0 0.05",
            "timing_pulley": "0.55 0 0.05",
        }
        for key, xyz in layout.items():
            mesh: trimesh.Trimesh = parts.CATALOG[key][2]()
            mesh.export(pkg / "meshes" / f"{key}.stl")
            print(f"demo_cell/{key}.stl: {len(mesh.faces):,} triangles")
            links.append(LINK.format(name=f"{key}_link", mesh=key, xyz=xyz))
        (pkg / "urdf" / "demo_cell.urdf").write_text(URDF.format(links="\n".join(links)))
        shutil.make_archive(str(HERE / "demo_cell_description"), "zip", tmp)
    print("demo_cell_description.zip written")


if __name__ == "__main__":
    main()
