# AI Mid-term Field Notes
- I have downloaded the mid-term starter code and I noticed that the asset labelled as a mountain is a pyramid and the asset labelled as a pyramid is a mountain.
- The code also didn't run on my linux machine right from the get go. I had to paste in the while loop fragment that I used while following along with the coursera vidoes.
- I'm thinking that I enable friction in pybullet somehow.
- Perhaps I can change the function for the motion of the robot

## Performance Optimization Session

- I did a deep dive into the performance bottlenecks in the codebase. The simulation was running slower than it needed to.

- First major find: the `ThreadedSim` class was creating a new multiprocessing Pool for every batch of creatures evaluated. This is expensive because spawning processes has overhead. Fixed it by creating the pool once in `__init__` and reusing it across all `eval_population()` calls. Added a `close()` method to clean up properly.

- Second optimization: URDF files were being written to disk for every creature evaluation. Switched to writing them to `/dev/shm` which is a RAM filesystem on Linux - much faster. Also added cleanup so the files get deleted right after PyBullet loads them.

- Simplified the `eval_population()` method significantly. The old code had this convoluted batching loop with nested lists. Replaced it with a single list comprehension and one `starmap` call. Much cleaner.

- Discovered a bug in the motor control code in `creature.py` - using `if` instead of `elif` for PULSE vs SINE motor types. This means both branches could execute. Haven't fixed this yet but noted it for later.

- Also noticed the fitness function I implemented (`climb_score`, `approach_score`, `height_time`, `penalty`) isn't actually being used by the GA loop. The `test_ga.py` still uses `get_distance_travelled()` for selection. Need to decide whether to switch over or if distance is actually what I want.

## Debug Screenshots

- Added a debug mode to `run_creature()`. When you pass `debug=True`, it captures screenshots every 240 simulation steps (about 1 second of sim time at 240fps).

- Had to figure out PyBullet's `getCameraImage()` - it works in headless DIRECT mode which is nice. The tricky part was that it returns a flat buffer, not a numpy array, so had to reshape it.

- Screenshots save to `debug_{sim_id}_{step}.png`. Camera follows the creature's position so you can see what it's doing over time.

- Added Pillow as a dependency for saving the images.

## Configuration Tweaks

- Bumped up the default iterations from 2400 to 9600 - gives creatures more time to show what they can do.
- Increased arena size to 40.
- Changed creature spawn position to `[5, 5, 10]` so they start on the slope of the mountain rather than directly above the peak.
- Increased population size to 30 and gene count to 5 for richer evolution.
- Pool size set to 8 to match my CPU cores.

## New GA Runner Script

- Got tired of running the GA through unittest. Created `src/run.py` as a proper standalone script with argparse for all the parameters.

- Added fitness plotting with matplotlib. After each run it saves a `fitness_plot.png` showing best and mean fitness over generations. Makes it much easier to see if evolution is actually working.

- Implemented plateau detection - if the best fitness doesn't change for 80 generations, the run stops early. No point wasting compute if we've converged.

- Added a `--use-fitness-score` flag to switch between `get_distance_travelled()` and the climbing-focused `fitness_score`. Still need to experiment with which one produces better climbers.

## URDF Path Fix

- Was getting errors loading the mountain URDF. The `setAdditionalSearchPath()` approach wasn't working reliably. Switched to using explicit relative paths like `./src/shapes/gaussian_pyramid.urdf` instead.

## Motor Amplitude Bug (Still Not Fixed)

- Dug deeper into why creatures aren't walking properly. Found that the motor amplitude (`control_amp`) is stored but never actually used in `get_output()`. The motors always output full strength (1, -1, or sin(phase)) regardless of what amplitude evolved.

- The fix should be simple - multiply the output by `self.amp` at the end of `get_output()`. But haven't implemented it yet.

- Also the motor frequency scale might be too low. With scale=1 (range 0-1), motors complete very few cycles during a simulation run. Might need to bump this up to scale=5 or higher.

## Next Steps

- Fix the motor amplitude bug - multiply output by self.amp
- Increase motor frequency scale for faster movement
- Fix the motor `if`/`elif` bug
- Consider switching GA selection to use `fitness_score` instead of `get_distance_travelled()`
- The scene (arena + mountain) is still being rebuilt for every creature evaluation - could optimize by only resetting the creature, not the whole world
- Look into the `point_mutate` function - the `amount` parameter is ignored, it's hardcoded to 0.1
