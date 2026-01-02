#!/usr/bin/env python3
"""
Genetic Algorithm for evolving creatures to climb a mountain.
"""

import argparse
import csv
import json
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from pathlib import Path

import population
import simulation
from simulation import Environment
import genome
import creature

from dataclasses import dataclass


@dataclass
class Gen:
    idx: int
    best: float
    mean: float
    navg_links: int


def has_plateaued(ngens: int, data: list[Gen]):
    if len(data) < ngens:
        return False
    has_plateaued = True
    last_n = data[(len(data) - ngens) :]
    for gen in last_n:
        if gen.best != last_n[0].best:
            has_plateaued = False
            return has_plateaued
    return has_plateaued


def save_generation_data(gen_data: list[Gen], filename="generations.csv"):
    """Save generation data to CSV."""
    if not gen_data:
        print("No data to save")
        return

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["generation", "best_fitness", "mean_fitness", "avg_links"])
        for g in gen_data:
            writer.writerow([g.idx, g.best, g.mean, g.navg_links])

    print(f"Saved generation data to {filename}")


def plot_fitness(gen_data: list[Gen], filename="fitness_plot.png"):
    """Plot best and mean fitness over generations."""
    if not gen_data:
        print("No data to plot")
        return

    generations = [g.idx for g in gen_data]
    best_fitness = [g.best for g in gen_data]
    mean_fitness = [g.mean for g in gen_data]

    plt.figure(figsize=(10, 6))
    plt.plot(generations, best_fitness, label="Best Fitness", linewidth=2)
    plt.plot(generations, mean_fitness, label="Mean Fitness", linewidth=2, alpha=0.7)

    plt.xlabel("Generation")
    plt.ylabel("Fitness")
    plt.title("Fitness Over Generations")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.savefig(filename, dpi=150)
    plt.close()
    print(f"Saved fitness plot to {filename}")


def run_ga(
    pop_size=10,
    gene_count=5,
    pool_size=8,
    generations=1000,
    iterations=4800,
    mutation_rate=0.1,
    shrink_rate=0.25,
    grow_rate=0.1,
    use_fitness_score=True,
    env=Environment.GAUSSIAN_PYRAMID,
):
    """Run the genetic algorithm."""

    # Create output directory structure
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path("data") / f"run_{timestamp}"
    elites_dir = run_dir / "elites"
    elites_dir.mkdir(parents=True, exist_ok=True)

    # Save hyperparameters
    hyperparams = {
        "pop_size": pop_size,
        "gene_count": gene_count,
        "pool_size": pool_size,
        "generations": generations,
        "iterations": iterations,
        "mutation_rate": mutation_rate,
        "shrink_rate": shrink_rate,
        "grow_rate": grow_rate,
        "use_fitness_score": use_fitness_score,
        "env": env.value,
        "timestamp": timestamp,
    }
    with open(run_dir / "hyperparameters.json", "w") as f:
        json.dump(hyperparams, f, indent=2)

    print(f"Output directory: {run_dir}")
    print(
        f"Starting GA: pop_size={pop_size}, gene_count={gene_count}, generations={generations}"
    )
    print(f"Environment: {env.value}")
    print(
        f"Using {'fitness_score' if use_fitness_score else 'distance_travelled'} for selection"
    )
    print("-" * 60)

    pop = population.Population(pop_size=pop_size, gene_count=gene_count)
    sim = simulation.ThreadedSim(pool_size=pool_size)

    gen_data: list[Gen] = []

    try:
        for gen in range(generations):
            # if has_plateaued(ngens=80, data=gen_data):
            #     break

            sim.eval_population(pop, iterations, env=env)

            # Get fitness values
            if use_fitness_score:
                fits = [cr.fitness_score for cr in pop.creatures]
            else:
                fits = [cr.get_distance_travelled() for cr in pop.creatures]

            links = [len(cr.get_expanded_links()) for cr in pop.creatures]

            g = Gen(
                idx=gen,
                best=np.max(fits),
                mean=np.mean(fits),
                navg_links=np.mean(links),
            )

            gen_data.append(g)

            # Print progress
            print(
                f"Gen {g.idx:4d} | "
                f"best: {g.best:7.3f} | "
                f"mean: {g.mean:7.3f} | "
                f"links: {np.mean(links):.1f} avg, {np.max(links)} max"
            )

            # Build fitness map for selection
            # Shift fitness to be positive if using fitness_score (can be negative)
            fit_values = np.array(fits)
            if np.min(fit_values) < 0:
                fit_values = fit_values - np.min(fit_values) + 0.1
            fit_map = population.Population.get_fitness_map(fit_values.tolist())

            # Create next generation
            new_creatures = []
            for _ in range(len(pop.creatures)):
                p1_ind = population.Population.select_parent(fit_map)
                p2_ind = population.Population.select_parent(fit_map)
                p1 = pop.creatures[p1_ind]
                p2 = pop.creatures[p2_ind]

                dna = genome.Genome.crossover(p1.dna, p2.dna)
                dna = genome.Genome.point_mutate(dna, rate=mutation_rate, amount=0.25)
                dna = genome.Genome.shrink_mutate(dna, rate=shrink_rate)
                dna = genome.Genome.grow_mutate(dna, rate=grow_rate)

                cr = creature.Creature(1)
                cr.update_dna(dna)
                new_creatures.append(cr)

            # Elitism: preserve best creature
            best_idx = np.argmax(fits)
            best_cr = pop.creatures[best_idx]
            elite = creature.Creature(1)
            elite.update_dna(best_cr.dna)
            new_creatures[0] = elite

            # Save elite
            filename = elites_dir / f"elite_{gen}.csv"
            genome.Genome.to_csv(best_cr.dna, str(filename))

            pop.creatures = new_creatures

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        sim.close()
        print("Done. Pool closed.")

    # Save generation data and plot
    save_generation_data(gen_data, filename=str(run_dir / "generations.csv"))
    plot_fitness(gen_data, filename=str(run_dir / "fitness_plot.png"))


def main():
    parser = argparse.ArgumentParser(description="Evolve creatures to climb a mountain")
    parser.add_argument("--pop-size", type=int, default=10, help="Population size")
    parser.add_argument("--gene-count", type=int, default=5, help="Initial gene count")
    parser.add_argument(
        "--pool-size", type=int, default=8, help="Multiprocessing pool size"
    )
    parser.add_argument(
        "--generations", type=int, default=1000, help="Number of generations"
    )
    parser.add_argument(
        "--iterations", type=int, default=2400, help="Simulation steps per creature"
    )
    parser.add_argument(
        "--mutation-rate", type=float, default=0.1, help="Point mutation rate"
    )
    parser.add_argument(
        "--shrink-rate", type=float, default=0.25, help="Shrink mutation rate"
    )
    parser.add_argument(
        "--grow-rate", type=float, default=0.1, help="Grow mutation rate"
    )
    parser.add_argument(
        "--use-fitness-score",
        action="store_true",
        help="Use fitness_score (climbing) instead of distance_travelled",
    )
    parser.add_argument(
        "--env",
        type=str,
        choices=["plane", "gaussian_pyramid"],
        default="gaussian_pyramid",
        help="Environment to run in (default: gaussian_pyramid)",
    )

    args = parser.parse_args()

    # Convert env string to Environment enum
    env = Environment(args.env)

    run_ga(
        pop_size=args.pop_size,
        gene_count=args.gene_count,
        pool_size=args.pool_size,
        generations=args.generations,
        iterations=args.iterations,
        mutation_rate=args.mutation_rate,
        shrink_rate=args.shrink_rate,
        grow_rate=args.grow_rate,
        # use_fitness_score=args.use_fitness_score,
        use_fitness_score=True,
        env=env,
    )


if __name__ == "__main__":
    main()
