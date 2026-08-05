"""
cli.py - Command-line interface for ros-mesh-preprocessor.

Single file:
    ros-mesh-preprocessor --input model.stl --output model_small.obj --triangles 5000
    ros-mesh-preprocessor --input model.stl --output model_small.obj --reduction 80

URDF pair (visual + collision):
    ros-mesh-preprocessor --input cell.stl --urdf-output ./output \
        --visual 20000 --collision 3000

Batch (folder):
    ros-mesh-preprocessor --folder ./cell_assets --output ./output --collision 5000

Exit codes:
    0  success
    1  a known, reported error (bad input, bad mesh, bad arguments combination
       caught at runtime rather than by argparse)
    2  argument parsing error (raised by argparse itself, e.g. missing a
       required flag)
"""

import argparse
import logging
import sys
from pathlib import Path

from optimizer import __version__
from optimizer.core import (
    SUPPORTED_FORMATS,
    clean_mesh,
    load_mesh,
    resolve_output_path,
    resolve_target_triangles,
    save_mesh,
    simplify_mesh,
)
from optimizer.metrics import build_report, print_report, save_report
from optimizer.urdf_export import export_urdf_pair

log = logging.getLogger("optimizer")


class CliError(Exception):
    """A known, user-facing error. Caught in main() and reported with exit code 1."""


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _process_single(args: argparse.Namespace) -> None:
    log.info("Loading '%s' …", args.input)
    mesh = load_mesh(args.input)
    mesh = clean_mesh(mesh)

    target = resolve_target_triangles(mesh, args.triangles, args.reduction)
    output_path = resolve_output_path(args.output, args.format)

    if args.dry_run:
        print(
            f"[dry-run] Would simplify '{args.input}' ({len(mesh.triangles):,} triangles) "
            f"→ {target:,} triangles"
        )
        print(f"[dry-run] Would write → '{output_path}'")
        if args.save_report:
            report_path = str(Path(output_path).with_suffix(".report.json"))
            print(f"[dry-run] Would save report → '{report_path}'")
        return

    log.info("Simplifying to %s triangles …", f"{target:,}")
    result, elapsed = simplify_mesh(mesh, target)

    save_mesh(result, output_path)
    log.info("Saved → '%s'", output_path)

    report = build_report(
        args.input,
        output_path,
        mesh,
        result,
        elapsed,
        compute_hausdorff=not args.skip_hausdorff,
    )
    print_report(report)

    if args.save_report:
        report_path = str(Path(output_path).with_suffix(".report.json"))
        save_report(report, report_path)


def _process_urdf(args: argparse.Namespace) -> None:
    if args.visual is None or args.collision is None:
        raise CliError("--urdf-output requires both --visual and --collision triangle counts.")

    if args.dry_run:
        mesh = load_mesh(args.input)
        stem = Path(args.input).stem
        out = Path(args.urdf_output)
        visual_path = out / f"{stem}_visual.stl"
        collision_path = out / f"{stem}_collision.stl"
        snippet_path = out / f"{stem}_urdf_snippet.xml"
        print(f"[dry-run] Would simplify '{args.input}' ({len(mesh.triangles):,} triangles)")
        print(f"[dry-run] Would write → '{visual_path}' ({args.visual:,} triangles)")
        print(f"[dry-run] Would write → '{collision_path}' ({args.collision:,} triangles)")
        print(f"[dry-run] Would write → '{snippet_path}'")
        return

    export_urdf_pair(
        input_path=args.input,
        output_dir=args.urdf_output,
        visual_triangles=args.visual,
        collision_triangles=args.collision,
        compute_hausdorff=not args.skip_hausdorff,
    )


def _process_batch(args: argparse.Namespace) -> None:
    folder = Path(args.folder)
    if not folder.is_dir():
        raise CliError(f"'{folder}' is not a directory.")

    out_dir = Path(args.output) if args.output else folder / "optimized"
    files = sorted(f for f in folder.iterdir() if f.suffix.lower() in SUPPORTED_FORMATS)

    if not files:
        raise CliError(f"No supported mesh files found in '{folder}'.")

    if args.dry_run:
        print(f"[dry-run] Would process {len(files)} mesh(es) in '{folder}' → '{out_dir}'")
        for f in files:
            print(f"[dry-run]   {f.name}")
        return

    out_dir.mkdir(parents=True, exist_ok=True)
    log.info("Found %d mesh(es) in '%s'. Processing …", len(files), folder)
    all_reports = []
    failures = 0

    for f in files:
        try:
            mesh = load_mesh(str(f))
            mesh = clean_mesh(mesh)
            target = resolve_target_triangles(mesh, args.triangles, args.reduction)
            result, elapsed = simplify_mesh(mesh, target)
            out_path = resolve_output_path(str(out_dir / f.name), args.format)
            save_mesh(result, out_path)
            report = build_report(
                str(f),
                out_path,
                mesh,
                result,
                elapsed,
                compute_hausdorff=not args.skip_hausdorff,
            )
            print_report(report)
            all_reports.append(report)
        except Exception as exc:
            log.warning("Skipping '%s': %s", f.name, exc)
            failures += 1

    if args.save_report:
        import json

        report_path = out_dir / "batch_report.json"
        with open(report_path, "w") as fh:
            json.dump(all_reports, fh, indent=2)
        log.info("Batch report saved → %s", report_path)

    if failures and not all_reports:
        raise CliError(f"All {failures} file(s) in the batch failed to process.")


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="ros-mesh-preprocessor",
        description=(
            "Mesh optimization toolkit for ROS2, MoveIt, and digital twins: "
            "decimation, URDF export, and geometric validation."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Reduce a single file to 5 000 triangles
  ros-mesh-preprocessor --input gear.stl --output gear_small.obj --triangles 5000

  # Reduce by percentage
  ros-mesh-preprocessor --input env.ply --output env_small.ply --reduction 80

  # Preview what a run would do without writing anything
  ros-mesh-preprocessor --input gear.stl --output gear_small.obj --triangles 5000 --dry-run

  # Generate URDF-ready visual + collision pair
  ros-mesh-preprocessor --input robot_cell.stl --urdf-output ./output \\
      --visual 20000 --collision 3000

  # Batch-process a whole folder, forcing OBJ output regardless of source format
  ros-mesh-preprocessor --folder ./cell_assets --output ./optimized \\
      --collision 5000 --format obj

  # Verbose logging for debugging a large batch job
  ros-mesh-preprocessor --folder ./cell_assets --collision 5000 -v
""",
    )

    p.add_argument("--version", action="version", version=f"ros-mesh-preprocessor {__version__}")

    # --- Input modes ---
    input_group = p.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--input", metavar="FILE", help="Single input mesh file.")
    input_group.add_argument("--folder", metavar="DIR", help="Folder of mesh files (batch mode).")

    # --- Output ---
    p.add_argument(
        "--output",
        metavar="PATH",
        help="Output file (single mode) or output directory (batch mode).",
    )
    p.add_argument(
        "--urdf-output",
        metavar="DIR",
        help="Output directory for URDF pair (activates URDF mode).",
    )
    p.add_argument(
        "--format",
        metavar="EXT",
        help="Force output format (e.g. 'obj', 'stl'), overriding the output "
        "path's extension. Applies to single and batch modes.",
    )

    # --- Target size ---
    size_group = p.add_mutually_exclusive_group()
    size_group.add_argument(
        "--triangles", type=int, metavar="N", help="Target triangle count (absolute)."
    )
    size_group.add_argument(
        "--reduction", type=float, metavar="PCT", help="Reduction percentage, e.g. 80 → keep 20%%."
    )

    # --- URDF pair sizes ---
    p.add_argument(
        "--visual", type=int, metavar="N", help="Triangle count for visual mesh (URDF mode)."
    )
    p.add_argument(
        "--collision",
        type=int,
        metavar="N",
        help="Triangle count for collision mesh (URDF / batch mode).",
    )

    # --- Extras ---
    p.add_argument(
        "--save-report",
        action="store_true",
        help="Persist a JSON performance/quality report alongside output.",
    )
    p.add_argument(
        "--skip-hausdorff",
        action="store_true",
        help="Skip Hausdorff distance computation (faster for large meshes).",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Print what would happen without reading targets or writing any files.",
    )
    p.add_argument(
        "-v",
        "--verbose",
        action="count",
        default=0,
        help="Increase logging verbosity. Repeat for more detail (-v, -vv).",
    )

    return p


def _configure_logging(verbosity: int) -> None:
    """Default shows progress (INFO); -v adds debug detail from dependencies too."""
    level = logging.DEBUG if verbosity >= 1 else logging.INFO
    logging.basicConfig(level=level, format="[%(name)s] %(message)s")
    if verbosity == 0:
        logging.getLogger("open3d").setLevel(logging.WARNING)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    _configure_logging(args.verbose)

    try:
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

    except CliError as exc:
        log.error("%s", exc)
        sys.exit(1)
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        log.error("%s", exc)
        sys.exit(1)


if __name__ == "__main__":
    main()
