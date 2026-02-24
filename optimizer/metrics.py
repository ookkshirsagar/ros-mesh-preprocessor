"""
metrics.py - Geometry validation and performance reporting.

Computes:
  - Triangle counts (before / after)
  - File size delta
  - Processing time
  - Hausdorff distance (geometric deviation estimate)
"""

import json
import time
from pathlib import Path

import numpy as np
import open3d as o3d


# ---------------------------------------------------------------------------
# Hausdorff distance
# ---------------------------------------------------------------------------

def hausdorff_distance(
    original: o3d.geometry.TriangleMesh,
    simplified: o3d.geometry.TriangleMesh,
    n_samples: int = 10_000,
) -> dict[str, float]:
    """
    Estimate one-sided and symmetric Hausdorff distances by sampling
    points on both surfaces and querying nearest-neighbour distances.

    Returns a dict with keys: max_mm, mean_mm, rms_mm
    (assumes mesh units are millimetres; adjust label if metres).
    """
    pcd_orig = original.sample_points_uniformly(number_of_points=n_samples)
    pcd_simp = simplified.sample_points_uniformly(number_of_points=n_samples)

    dists_orig_to_simp = np.asarray(
        pcd_orig.compute_point_cloud_distance(pcd_simp)
    )
    dists_simp_to_orig = np.asarray(
        pcd_simp.compute_point_cloud_distance(pcd_orig)
    )

    all_dists = np.concatenate([dists_orig_to_simp, dists_simp_to_orig])

    return {
        "hausdorff_max": float(np.max(all_dists)),
        "hausdorff_mean": float(np.mean(all_dists)),
        "hausdorff_rms": float(np.sqrt(np.mean(all_dists**2))),
    }


# ---------------------------------------------------------------------------
# Performance report
# ---------------------------------------------------------------------------

def build_report(
    input_path: str,
    output_path: str,
    original_mesh: o3d.geometry.TriangleMesh,
    result_mesh: o3d.geometry.TriangleMesh,
    processing_time_sec: float,
    compute_hausdorff: bool = True,
    hausdorff_samples: int = 10_000,
) -> dict:
    """Build a structured performance/quality report dict."""
    orig_tri = len(original_mesh.triangles)
    red_tri = len(result_mesh.triangles)
    reduction_pct = round((1 - red_tri / orig_tri) * 100, 2) if orig_tri > 0 else 0.0

    in_size = Path(input_path).stat().st_size / (1024 * 1024)
    out_size = Path(output_path).stat().st_size / (1024 * 1024) if Path(output_path).exists() else 0.0

    report = {
        "input_file": str(input_path),
        "output_file": str(output_path),
        "original_triangles": orig_tri,
        "reduced_triangles": red_tri,
        "reduction_percent": reduction_pct,
        "file_size_before_mb": round(in_size, 3),
        "file_size_after_mb": round(out_size, 3),
        "processing_time_sec": round(processing_time_sec, 4),
    }

    if compute_hausdorff:
        h = hausdorff_distance(original_mesh, result_mesh, n_samples=hausdorff_samples)
        report.update(h)

    return report


def print_report(report: dict) -> None:
    """Pretty-print the report to stdout."""
    sep = "─" * 52
    print(f"\n{sep}")
    print("  Mesh Preprocessing Report")
    print(sep)
    print(f"  Input         : {report['input_file']}")
    print(f"  Output        : {report['output_file']}")
    print(f"  Triangles     : {report['original_triangles']:,}  →  {report['reduced_triangles']:,}")
    print(f"  Reduction     : {report['reduction_percent']}%")
    print(f"  File size     : {report['file_size_before_mb']} MB  →  {report['file_size_after_mb']} MB")
    print(f"  Time          : {report['processing_time_sec']} s")
    if "hausdorff_max" in report:
        print(f"  Hausdorff max : {report['hausdorff_max']:.6f} (mesh units)")
        print(f"  Hausdorff RMS : {report['hausdorff_rms']:.6f} (mesh units)")
    print(sep + "\n")


def save_report(report: dict, path: str) -> None:
    """Persist the report as JSON."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w") as fh:
        json.dump(report, fh, indent=2)
    print(f"[metrics] Report saved → {p}")
