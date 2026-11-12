"""
Upload the browser GUI to a Hugging Face Space.

Assembles space/ + optimizer/ + examples/gui/ into one folder and uploads it with
huggingface_hub (which stores the binary example meshes correctly). Creates the
Space on first run.

Env: HF_TOKEN (write token), HF_SPACE ("username/space-name").
"""

import os
import shutil
import tempfile
from pathlib import Path

from huggingface_hub import HfApi

ROOT = Path(__file__).resolve().parents[2]
space_id = os.environ["HF_SPACE"]
api = HfApi(token=os.environ["HF_TOKEN"])

with tempfile.TemporaryDirectory() as tmp:
    build = Path(tmp)
    shutil.copytree(ROOT / "space", build, dirs_exist_ok=True)
    shutil.copytree(
        ROOT / "optimizer", build / "optimizer", ignore=shutil.ignore_patterns("__pycache__")
    )
    examples = build / "examples" / "gui"
    examples.mkdir(parents=True)
    for f in (ROOT / "examples" / "gui").glob("*"):
        if f.suffix in (".stl", ".zip"):
            shutil.copy2(f, examples / f.name)

    api.create_repo(space_id, repo_type="space", space_sdk="gradio", exist_ok=True)
    api.upload_folder(
        repo_id=space_id,
        repo_type="space",
        folder_path=build,
        commit_message=f"Deploy {os.environ.get('GITHUB_SHA', 'local')[:7]} from GitHub",
        delete_patterns=["*"],  # replace the Space contents with this build
    )
print(f"Deployed to https://huggingface.co/spaces/{space_id}")
