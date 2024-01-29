import uuid
import manual_eval
import os
import subprocess
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import multiprocessing
from concurrent.futures import ThreadPoolExecutor
import time
from tqdm import tqdm
import sys

def run_experiment(experiment_conf, x):
    file_name = str(uuid.uuid4())
    experiment_conf_tmp = experiment_conf.replace('insert_filename_here', sys.argv[1] + '_' + str(x[0]) + '_' + str(x[2]) + '_' + str(x[1]))
    experiment_conf_tmp = experiment_conf_tmp.replace('insert_seed_here', str(x[1]))
    experiment_conf_tmp = experiment_conf_tmp.replace('insert_noise_here', str(x[2]))
    experiment_conf_tmp = experiment_conf_tmp.replace('insert_noise_deg_here', str(x[3]))
    experiment_conf_tmp = experiment_conf_tmp.replace('insert_quantity_here', str(x[0]))
    if (x[2] > 0):
        experiment_conf_tmp = experiment_conf_tmp.replace('pkt_drop_here', str(0.1))
    else:
        experiment_conf_tmp = experiment_conf_tmp.replace('pkt_drop_here', str(0))

    with open('experiments/' + file_name + ".autogen", 'w') as file:
        file.write(experiment_conf_tmp)
    
    subprocess.run(['argos3', '-c', 'experiments/' + file_name + '.autogen'], stdout=subprocess.PIPE)
    os.remove('experiments/' + file_name + '.autogen')

# define worker function before a Pool is instantiated
def work(x):
    run_experiment(experiment_conf, x)
    return 0

def run_experiment2(experiment_conf, x):
    file_name = str(uuid.uuid4())
    experiment_conf_tmp = experiment_conf.replace('insert_filename_here', sys.argv[2] + '_' + str(x[0]) + '_' + str(x[2]) + '_' + str(x[1]))
    experiment_conf_tmp = experiment_conf_tmp.replace('insert_seed_here', str(x[1]))
    experiment_conf_tmp = experiment_conf_tmp.replace('insert_quantity_here', str(x[0]))
    experiment_conf_tmp = experiment_conf_tmp.replace('insert_cognitive_speed_here', str(x[2]))

    if (x[2] == 0):
        experiment_conf_tmp = experiment_conf_tmp.replace('insert_duration_here', str(400))
        experiment_conf_tmp = experiment_conf_tmp.replace('insert_ticks_here', str(20))
        experiment_conf_tmp = experiment_conf_tmp.replace('insert_halve_here', 'true')
    else:
        experiment_conf_tmp = experiment_conf_tmp.replace('insert_duration_here', str(200))
        experiment_conf_tmp = experiment_conf_tmp.replace('insert_ticks_here', str(10))
        experiment_conf_tmp = experiment_conf_tmp.replace('insert_halve_here', 'false')

    if (x[2] > 3):
        experiment_conf_tmp = experiment_conf_tmp.replace('pkt_drop_here', str(0.0))
    else:
        experiment_conf_tmp = experiment_conf_tmp.replace('pkt_drop_here', str(0.1))

    with open('experiments/' + file_name + ".autogen", 'w') as file:
        file.write(experiment_conf_tmp)
    
    subprocess.run(['argos3', '-c', 'experiments/' + file_name + '.autogen'], stdout=subprocess.PIPE)
    os.remove('experiments/' + file_name + '.autogen')

# define worker function before a Pool is instantiated
def work2(x):
    run_experiment2(experiment_conf, x)
    return 0
        
if __name__ == "__main__":
    file_name = sys.argv[1]
    experiment_conf = ""
    with open('experiments/experiment.argos', 'r') as file:
        experiment_conf = file.read()

    noise = [0.2]
    noise_deg = [0, 2, 5, 10, 20]
    sizes = [22, 56, 113, 169, 225]
    exps = []
    for i in range(len(sizes)):
        for z in range(len(noise)):
            for j in range(1, 76):
                exps.append((sizes[i], sizes[i]*10+j, noise[z], noise_deg[z]))

    with ThreadPoolExecutor(max_workers=8) as executor:
        res = list(tqdm(executor.map(work, exps), total=len(exps)))

    file_name = sys.argv[2]
    experiment_conf = ""
    with open('experiments/experiment2.argos', 'r') as file:
        experiment_conf = file.read()

    cog_speed = [0, 1, 2, 10, 20, 40]
    sizes = [22, 56, 113, 169, 225]
    exps = []
    for i in range(len(sizes)):
        for z in range(len(cog_speed)):
            for j in range(1, 76):
                exps.append((sizes[i], sizes[i]*100+j, cog_speed[z]))


    with ThreadPoolExecutor(max_workers=8) as executor:
        res = list(tqdm(executor.map(work2, exps), total=len(exps)))
