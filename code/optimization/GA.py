import random
import numpy as np
from planner import simulate_order_once
from optimization.Localoperator import crossover
from optimization.Localoperator import mutate

def genetic_algorithm(players, goals, power, env, pop_size=40, generations=80):

    import time
    t_start = time.time()

    N = len(players)

    best_cost = float("inf")
    best_order = None
    best_metrics = None
    best_paths = None
    gen_valid = None

    best_hist = []
    eval_hist = []
    valid_flags = []
    # fitness wrapper
    def fitness(order):
        nonlocal best_cost, best_order, best_metrics, best_paths, gen_valid
        C, S, W, G, B, E, paths, valid = simulate_order_once(order, players, goals, power, env.copy())

        if E < best_cost:
            best_cost = E
            best_order = order
            best_metrics = (C, S, W, G, B, E)
            best_paths = paths
            gen_valid = valid
        return E

    # initialize population
    population = [tuple(np.random.permutation(N)) for _ in range(pop_size)]
    scores = [fitness(o) for o in population]
    
    valid_flags.append(gen_valid)

    best_hist.append(best_cost)
    eval_hist.append(0)

    # GA loop
    for g in range(1, generations + 1):

        scored = sorted(zip(scores, population), key=lambda x: x[0])
        survivors = [o for _, o in scored[:pop_size//2]]

        # elitism
        if best_order not in survivors:
            survivors[0] = best_order

        # children
        children = []
        while len(children) < pop_size - len(survivors):
            p1, p2 = random.sample(survivors, 2)
            c = crossover(p1, p2)
            c = mutate(c)
            children.append(c)

        population = survivors + children

        scores = [fitness(o) for o in population]
        valid_flags.append(gen_valid)
        best_hist.append(best_cost)
        eval_hist.append(g)

    total_time = time.time() - t_start

    return best_order, best_metrics, best_paths, eval_hist, best_hist, valid_flags, total_time
