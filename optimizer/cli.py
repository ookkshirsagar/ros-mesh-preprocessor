"""
cli.py - Command-line interface for ros-mesh-preprocessor.

Single file:
    python -m optimizer.cli --input model.stl --output model_small.obj --triangles 5000
    python -m optimizer.cli --input model.stl --output model_small.obj --reduction 80

URDF pair (visual + collision):
    python -m optimizer.cli --input cell.stl --urdf-output ./output \
        --visual 20000 --collision 3000

Batch (folder):
    python -m optimizer.cli --folder ./cell_assets --output ./output --collision 5000
"""

import argparse
import sys
from pathlib import Path

from optimizer.core import (
    clean_mesh,
    load_mesh,
    resolve_target_triangles,
    save_mesh,
    simplify_mesh,
)
from optimizer.metrics import build_report, print_report, save_report
from optimizer.urdf_export import export_urdf_pair


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _process_single(args: argparse.Namespace) -> None:
    print(f"[cli] Loading '{args.input}' …")
    mesh = load_mesh(args.input)
    mesh = clean_mesh(mesh)

    target = resolve_target_triangles(mesh, args.triangles, args.reduction)
    print(f"[cli] Simplifying to {target:,} triangles …")
    result, elapsed = simplify_mesh(mesh, target)

    save_mesh(result, args.output)
    print(f"[cli] Saved → '{args.output}'")

    report = build_report(
        args.input, args.output, mesh, result, elapsed,
        compute_hausdorff=not args.skip_hausdorff,
    )
    print_report(report)

    if args.save_report:
        report_path = str(Path(args.output).with_suffix(".report.json"))
        save_report(report, report_path)


def _process_urdf(args: argparse.Namespace) -> None:
    if args.visual is None or args.collision is None:
        print("[cli] Error: --urdf-output requires both --visual and --collision triangle counts.")
        sys.exit(1)
    export_urdf_pair(
        input_path=args.input,
        output_dir=args.urdf_output,
        visual_triangles=args.visual,
        collision_triangles=args.collision,
        compute_hausdorff=not args.skip_hausdorff,
    )


def _process_batch(args: argparse.Namespace) -> None:
    folder = Path(args.folder)
    out_dir = Path(args.output) if args.output else folder / "optimized"
    out_dir.mkdir(parents=True, exist_ok=True)

    supported = {".stl", ".obj", ".ply", ".off", ".glb", ".gltf"}
    files = [f for f in folder.iterdir() if f.suffix.lower() in supported]

    if not files:
        print(f"[cli] No supported mesh files found in '{folder}'.")
        sys.exit(1)

    print(f"[cli] Found {len(files)} mesh(es) in '{folder}'. Processing …\n")
    all_reports = []

    for f in sorted(files):
        try:
            mesh = load_mesh(str(f))
            mesh = clean_mesh(mesh)
            target = resolve_target_triangles(mesh, args.triangles, args.reduction)
            result, elapsed = simplify_mesh(mesh, target)
            out_path = str(out_dir / f.name)
            save_mesh(result, out_path)
            report = build_report(
                str(f), out_path, mesh, result, elapsed,
                compute_hausdorff=not args.skip_hausdorff,
            )
            print_report(report)
            all_reports.append(report)
        except Exception as exc:
            print(f"[cli] WARNING: Skipping '{f.name}' — {exc}")

    if args.save_report:
        import json
        report_path = out_dir / "batch_report.json"
        with open(report_path, "w") as fh:
            json.dump(all_reports, fh, indent=2)
        print(f"[cli] Batch report saved → {report_path}")


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="ros-mesh-preprocessor",
        description=(
            "Geometry preprocessing toolkit for ROS2, MoveIt, and "
            "industrial robot cell modelling."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Reduce a single file to 5 000 triangles
  python -m optimizer.cli --input gear.stl --output gear_small.obj --triangles 5000

  # Reduce by percentage
  python -m optimizer.cli --input env.ply --output env_small.ply --reduction 80

  # Generate URDF-ready visual + collision pair
  python -m optimizer.cli --input robot_cell.stl --urdf-output ./output \\
      --visual 20000 --collision 3000

  # Batch-process a whole folder
  python -m optimizer.cli --folder ./cell_assets --output ./optimized --collision 5000
""",
    )

    # --- Input modes ---
    input_group = p.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--input", metavar="FILE", help="Single input mesh file.")
    input_group.add_argument("--folder", metavar="DIR", help="Folder of mesh files (batch mode).")

    # --- Output ---
    p.add_argument("--output", metavar="PATH",
                   help="Output file (single mode) or output directory (batch mode).")
    p.add_argument("--urdf-output", metavar="DIR",
                   help="Output directory for URDF pair (activates URDF mode).")

    # --- Target size ---
    size_group = p.add_mutually_exclusive_group()
    size_group.add_argument("--triangles", type=int, metavar="N",
                            help="Target triangle count (absolute).")
    size_group.add_argument("--reduction", type=float, metavar="PCT",
                            help="Reduction percentage, e.g. 80 → keep 20%%.")

    # --- URDF pair sizes ---
    p.add_argument("--visual", type=int, metavar="N",
                   help="Triangle count for visual mesh (URDF mode).")
    p.add_argument("--collision", type=int, metavar="N",
                   help="Triangle count for collision mesh (URDF / batch mode).")

    # --- Extras ---
    p.add_argument("--save-report", action="store_true",
                   help="Persist a JSON performance/quality report alongside output.")
    p.add_argument("--skip-hausdorff", action="store_true",
                   help="Skip Hausdorff distance computation (faster for large meshes).")

    return p


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    # ---- Routing ----
    if args.folder:
        if args.triangles is None and args.reduction is None and args.collision is None:
            parser.error("Batch mode requires --triangles, --reduction, or --collision.")
        # In batch mode --collision is a shorthand for --triangles
        if args.collision and args.triangles is None and args.reduction is None:
            args.triangles = args.collision
        _process_batch(args)

    elif args.urdf_output:
        _process_urdf(args)

    else:
        if args.output is None:
            parser.error("--output is required in single-file mode.")
        if args.triangles is None and args.reduction is None:
            parser.error("Single-file mode requires --triangles or --reduction.")
        _process_single(args)


if __name__ == "__main__":
    main()
