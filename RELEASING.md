# Releasing

Releases are fully automated by [`.github/workflows/release.yaml`](.github/workflows/release.yaml):
pushing a `vX.Y.Z` tag runs the tests, builds the sdist and wheel, publishes them to
[PyPI](https://pypi.org/project/urdf-mesh-optimizer/) and creates a GitHub Release
whose notes are that version's section of [CHANGELOG.md](CHANGELOG.md).

## One-time setup (repository owner)

PyPI publishing uses [Trusted Publishing](https://docs.pypi.org/trusted-publishers/)
(OpenID Connect), so no API token is stored in the repository.

1. Sign in at <https://pypi.org> (create an account and enable 2FA if needed).
2. Go to **Your account → Publishing → Add a new pending publisher** and enter:
   - PyPI project name: `urdf-mesh-optimizer`
   - Owner: `ookkshirsagar`
   - Repository name: `urdf-mesh-optimizer`
   - Workflow name: `release.yaml`
   - Environment name: `pypi`
3. On GitHub: **Settings → Environments → New environment** named `pypi`.
   Optionally add yourself as a required reviewer so every publish needs a click.

The first successful run creates the project on PyPI. After that, the pending
publisher becomes a normal trusted publisher.

## Cutting a release

1. Make sure `main` is green in CI.
2. Set the version in `optimizer/__init__.py` (`__version__ = "X.Y.Z"`).
3. In `CHANGELOG.md`, move the `[Unreleased]` entries under `## [X.Y.Z] - YYYY-MM-DD`
   and update the compare links at the bottom.
4. Commit and push to `main`, then either click **Actions → Release → Run workflow**
   on `main` (the workflow creates the `vX.Y.Z` tag itself), or tag and push:

   ```bash
   git commit -am "release: vX.Y.Z"
   git tag vX.Y.Z
   git push origin main vX.Y.Z
   ```

5. Watch the **Release** workflow in the Actions tab. It fails early if the tag
   doesn't match `__version__`.

## After the first release

Add a PyPI badge to the README and switch the Quick start to `pip install urdf-mesh-optimizer`:

```markdown
[![PyPI](https://img.shields.io/pypi/v/urdf-mesh-optimizer.svg)](https://pypi.org/project/urdf-mesh-optimizer/)
```
