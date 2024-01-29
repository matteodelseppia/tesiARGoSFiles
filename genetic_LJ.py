import uuid
import os
import subprocess
import manual_eval
import numpy as np
import sys

from pymoo.algorithms.soo.nonconvex.ga import GA
from pymoo.optimize import minimize
from multiprocessing.pool import ThreadPool
from pymoo.core.problem import StarmapParallelization
from pymoo.termination.default import DefaultSingleObjectiveTermination
from pymoo.core.problem import ElementwiseProblem
import matplotlib.pyplot as plt

experiment_conf = ""
with open('experiments/flocking_exp_LJ.argos', 'r') as file:
    experiment_conf = file.read()

def run_experiment(seed, gain, LJ_a, LJ_b):
    file_name = str(uuid.uuid4())
    experiment_conf_tmp = experiment_conf.replace('insert_filename_here', file_name)
    experiment_conf_tmp = experiment_conf_tmp.replace('insert_gain_here', gain)
    experiment_conf_tmp = experiment_conf_tmp.replace('LJ_a_here', LJ_a)
    experiment_conf_tmp = experiment_conf_tmp.replace('LJ_b_here', LJ_b)
    experiment_conf_tmp = experiment_conf_tmp.replace('insert_seed_here', seed)

    with open('experiments/' + file_name + ".experiment", 'w') as file:
        file.write(experiment_conf_tmp)

    subprocess.run(['argos3', '-c', 'experiments/' + file_name + '.experiment'], stdout=subprocess.PIPE)
    relative_connectivity, avg_neighbor_distance, v_mismatch, target_dist = manual_eval.evaluate(file_name + '.experiment')
    try:
        result = 100000*(1 - np.min(relative_connectivity[-50:])) + 1000*np.mean((avg_neighbor_distance[-50:])) + np.mean(target_dist[-50:])
    except:
        result = 100000000

    os.remove('experiments/' + file_name + '.experiment')
    os.remove(file_name + '.experiment')
    return result
    

class OptimizePotential(ElementwiseProblem):

    def __init__(self, **kwargs):
        super().__init__(n_var=3, n_obj=1, xl=np.array([0, 0, 0]), xu=np.array([100, 12, 12]), elementwise_evaluation=True, **kwargs)

    def _evaluate(self, x, out, *args, **kwargs):
        val = 0
        for i in range(10):
            val += run_experiment(str(i+1), "%.2f" % x[0],"%.2f" % x[1],"%.2f" % (x[2]))
        out["F"] = val/10

# initialize the thread pool and create the runner
n_threads = 4
pool = ThreadPool(n_threads)
runner = StarmapParallelization(pool.starmap)

termination = DefaultSingleObjectiveTermination(
    xtol=1e-6,
    cvtol=1e-6,
    ftol=1e-6,
    period=10,
    n_max_gen=100
)

# define the problem by passing the starmap interface of the thread pool
problem = OptimizePotential(elementwise_runner=runner)

algorithm = GA(
    pop_size=int(sys.argv[1]),
    termination=termination,
    eliminate_duplicates=True,
    seed=1)
res = minimize(problem,
               algorithm,
               termination,
              seed=1,
              verbose=True)


pool.close()

print(res.F)
print(res.X)
