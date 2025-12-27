from simulation import Simulation
from creature import Creature

def main():
    sim = Simulation()
    c = Creature(1)
    sim.run_creature(c, debug=True)

    




if __name__ == "__main__":
    main()
