# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a genetic algorithm project that evolves virtual creatures in a PyBullet physics simulation. Creatures are represented as URDF robots with cylindrical link bodies and revolute joints, controlled by motor outputs (sine or pulse waveforms). The fitness function is distance traveled.

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

- **simulation.py**: `Simulation` runs headless PyBullet physics. `ThreadedSim` uses multiprocessing Pool for parallel evaluation (platform-dependent).

### Genetic Encoding

Each gene is a numpy array mapping to 17 parameters (link geometry, joint configuration, motor control). Key gene spec fields:
- `link-length`, `link-radius`, `link-mass`: Physical properties
- `joint-parent`, `joint-axis-xyz`, `joint-origin-*`: Joint configuration
- `control-waveform`, `control-amp`, `control-freq`: Motor control

### Evolution Loop Pattern

The GA follows a standard pattern in `test_ga.py`:
1. Evaluate population fitness (distance traveled)
2. Select parents via fitness-proportionate selection
3. Apply crossover and mutations (point, shrink, grow)
4. Preserve elite individual (saves to `elite_N.csv`)

### DNA Persistence

Genomes serialize to/from CSV files using `Genome.to_csv()` and `Genome.from_csv()`. Elite creatures are saved as `elite_<iteration>.csv`.
