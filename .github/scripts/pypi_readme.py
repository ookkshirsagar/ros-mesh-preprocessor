"""
Rewrite relative links in README.md to absolute GitHub URLs pinned to a git ref,
so images and links render on the PyPI project page (PyPI cannot resolve
repository-relative paths). Run in CI just before `python -m build`.

Usage: python .github/scripts/pypi_readme.py <owner/repo> <ref> [README.md]
"""

import re
import sys
from pathlib import Path

repo, ref = sys.argv[1], sys.argv[2]
readme = Path(sys.argv[3] if len(sys.argv) > 3 else "README.md")
raw = f"https://raw.githubusercontent.com/{repo}/{ref}/"
blob = f"https://github.com/{repo}/blob/{ref}/"
IMAGE_EXT = (".png", ".jpg", ".jpeg", ".gif", ".svg", ".webp")


def absolute(path: str) -> str:
    if re.match(r"^([a-z][a-z0-9+.-]*:|#|/)", path, re.I):
        return path  # already absolute, an anchor, or site-rooted
    base = raw if path.lower().split("#")[0].endswith(IMAGE_EXT) else blob
    return base + path.removeprefix("./")


text = readme.read_text(encoding="utf-8")
text = re.sub(r"(\]\()([^)\s]+)", lambda m: m.group(1) + absolute(m.group(2)), text)
text = re.sub(r'((?:src|href)=")([^"]+)', lambda m: m.group(1) + absolute(m.group(2)), text)
readme.write_text(text, encoding="utf-8")
print(f"Rewrote relative links in {readme} -> {repo}@{ref}")
