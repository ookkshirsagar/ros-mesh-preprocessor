"""Hugging Face Space entry point: the urdf-mesh-optimizer browser GUI with upload limits."""

from optimizer.gui import launch

launch(max_upload_mb=50)
