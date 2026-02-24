# Contributing to ros-mesh-preprocessor

Thank you for considering a contribution. This is a robotics-focused tool and contributions that keep it practical and purposeful are most welcome.

---

## What we welcome

- Bug fixes and edge-case handling (e.g. degenerate mesh inputs)
- New simplification modes (e.g. voxel-based decimation as an alternative to QEM)
- ROS2 / MoveIt integration examples
- Performance improvements for large batch jobs
- Better Hausdorff sampling strategies
- Documentation improvements

## What to discuss first

Open an issue before starting work on:
- New CLI flags or modes that change existing behaviour
- Changes to the JSON report schema (downstream tools may depend on it)
- Adding new dependencies

---

## Getting started

```bash
git clone https://github.com/ookkshirsagar/ros-mesh-preprocessor.git
cd ros-mesh-preprocessor

python3 -m venv venv
source venv/bin/activate      # Windows: venv\Scripts\activate

pip install --upgrade pip
pip install -r requirements.txt
pip install black flake8       # dev tools
```

---

## Making a change

1. Fork the repo and create a branch from `main`:
   ```bash
   git checkout -b feat/my-feature
   ```

2. Make your changes. Keep commits focused,  one logical change per commit.

3. Run the test suite and make sure everything passes:
   ```bash
   pytest tests/ -v
   ```

4. Format and lint:
   ```bash
   black optimizer/ tests/
   flake8 optimizer/ tests/
   ```

5. Open a Pull Request against `main` with a clear description of what the change does and why.

---

## Code style

- **Black** for formatting (line length 99)
- **Flake8** for linting, no unused imports, no bare `except`
- Type hints on all public functions
- Docstrings on all public functions and classes

---

## Reporting bugs

Please include:
- OS and Python version
- The command you ran
- The full error message / traceback
- The mesh file format (STL / OBJ / etc.) and approximate triangle count

---

## License

By contributing you agree that your contributions will be licensed under the [MIT License](LICENSE).