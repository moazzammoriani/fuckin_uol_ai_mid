# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a genetic algorithm project that evolves virtual creatures in a PyBullet physics simulation. Creatures are represented as URDF robots with cylindrical link bodies and revolute joints, controlled by motor outputs (sine or pulse waveforms). Creatures evolve to climb a mountain terrain, with fitness based on height gained and approach to peak.

## Commands

### Setup
```bash
uv sync  # Install dependencies (uses uv package manager)
```

### Running Tests
```bash
cd src && python -m unittest test_genome.py      # Genome tests
cd src && python -m unittest test_creature.py    # Creature tests
cd src && python -m unittest test_simulation.py  # Simulation tests
cd src && python -m unittest test_population.py  # Population tests
```

### Running the Genetic Algorithm
```bash
# Multi-threaded version (Linux/Intel Mac with Python <=3.7 only)
cd src && python test_ga.py

# Single-threaded version (Windows, M1 Mac, or Intel Mac with Python >3.7)
cd src && python test_ga_no_threads.py
```

### Visualizing Evolved Creatures
```bash
# Real-time visualization with GUI
cd src && python realtime_from_csv.py <elite_N.csv>

# Headless evaluation
cd src && python offline_from_csv.py <elite_N.csv>
```

## Architecture

### Core Components (in `src/`)

- **genome.py**: Defines the genetic encoding. `Genome` class handles DNA operations (random generation, crossover, mutation). `URDFLink` converts gene dictionaries to URDF XML elements.

- **creature.py**: `Creature` wraps a genome and provides URDF generation via `to_xml()`. `Motor` class produces control signals (pulse or sine wave) for joint actuation.

- **population.py**: `Population` manages a collection of creatures. Provides fitness-proportionate selection via `get_fitness_map()` and `select_parent()`.

- **simulation.py**: `Simulation` runs headless PyBullet physics with arena and mountain terrain. `ThreadedSim` uses a persistent multiprocessing Pool for parallel evaluation. URDF files are written to `/dev/shm` (RAM) and deleted after loading.

### Genetic Encoding

Each gene is a numpy array mapping to 17 parameters (link geometry, joint configuration, motor control). Key gene spec fields:
- `link-length`, `link-radius`, `link-mass`: Physical properties
- `joint-parent`, `joint-axis-xyz`, `joint-origin-*`: Joint configuration
- `control-waveform`, `control-amp`, `control-freq`: Motor control

### Fitness Scoring

The simulation computes a multi-factor fitness score in `Simulation.run_creature()`:
- `climb_score`: Maximum height gained above base (weight: 1.0)
- `approach_score`: Horizontal distance moved toward peak at origin (weight: 0.5)
- `height_time`: Average height over simulation (weight: 0.1)
- `penalty`: -1.0 if creature falls off terrain

**Note:** Currently `test_ga.py` uses `get_distance_travelled()` for selection, not `fitness_score`.

### Evolution Loop Pattern

The GA follows a standard pattern in `test_ga.py`:
1. Evaluate population fitness via `ThreadedSim.eval_population()`
2. Select parents via fitness-proportionate selection
3. Apply crossover and mutations (point, shrink, grow)
4. Preserve elite individual (saves to `elite_N.csv`)
5. Call `sim.close()` when done to clean up the process pool

### DNA Persistence

Genomes serialize to/from CSV files using `Genome.to_csv()` and `Genome.from_csv()`. Elite creatures are saved as `elite_<iteration>.csv`.
