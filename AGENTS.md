# Repository Guidelines

## Project Structure & Module Organization
- Core code lives in `src/`: `genome.py` (gene specs, mutation/crossover, CSV I/O), `creature.py` (URDF assembly, motors, distance tracking), `simulation.py` (PyBullet runs, threaded helper), and `population.py` (GA population + selection).
- Supporting scripts: `starter.py` (minimal PyBullet setup), `offline_from_csv.py`/`realtime_from_csv.py` for loading DNA from CSV, `prepare_shapes.py` for asset prep, and `motor_test.py` for quick motor checks.
- Assets are in `src/shapes/` (URDF/OBJ terrains). Temporary URDFs (e.g., `test.urdf`, `temp*.urdf`) are written to the repo root when running sims.

## Environment, Build, and Run Commands
- Target Python 3.12 (see `pyproject.toml`). Create a venv: `python -m venv .venv && source .venv/bin/activate`.
- Install deps: `pip install -U ipython numpy pybullet` (or `uv sync` if you use uv and its lockfile).
- Run a quick sim loop: `python src/starter.py` (GUI) or `python -m src.offline_from_csv your_dna.csv` to replay a saved genome headlessly.
- Entry stub: `python main.py` currently only prints a greeting—use scripts above for physics.

## Coding Style & Naming Conventions
- Follow existing Python style: 4-space indentation, snake_case for functions/modules, CapWords for classes (e.g., `Creature`, `Simulation`). Keep NumPy and PyBullet APIs idiomatic.
- Avoid broad mutation side effects; methods often cache computed attributes (`Creature.get_expanded_links`, `get_motors`), so reset related fields when DNA changes.
- Keep URDF/XML generation deterministic; prefer explicit field names when adding genome spec entries.

## Testing Guidelines
- Tests use the standard library’s `unittest` and live in `src/test_*.py`. Run all tests with `python -m unittest discover -s src -p 'test_*.py'`.
- Some tests write small artifacts (`test.csv`, temporary URDFs) to the repo root; clean up if running repeatedly.
- PyBullet tests default to DIRECT mode, but GPU/GUI drivers can affect behavior—ensure PyBullet is installed correctly before filing failures.

## Commit & Pull Request Guidelines
- There is no existing history; use concise, present-tense commits (e.g., `Add genome crossover bounds`). Reference issues where applicable.
- For PRs, summarize the change, list test commands/results, and note any PyBullet/environment assumptions (GUI vs DIRECT, asset paths). Include screenshots/GIFs only when a visual change is relevant.
