import numpy as np
from scipy.spatial import distance
import seaborn as sns
import matplotlib.pyplot as plt
import sys
import math

# script to compute graphs for:
# average nearest neighbor distance across time
# average target distance across time
# relative connettivity across time
# cohesion radius across time
# normalized energy deviation across time

lattice_scale = 1.25
max_distance = 1
max_distance = max_distance*lattice_scale
d = max_distance/lattice_scale
arena_len = 30
cell_len = 2
num_cells = (30/cell_len)**2

def compute_entropy(positions):
    N = len(positions)
    cells = {}
    for i in range(N):
        cell_id = (math.floor(positions[i][0]), math.floor(positions[i][1]))
        if cell_id in cells:
            cells[cell_id] += 1
        else:
            cells[cell_id] = 1
    
    loc_entropy = 0
    for key1 in cells:
        pi = cells[key1]/N
        denom_local_entropy = 0
        for key2 in cells:
            is_neighbor = 0
            if (abs(key1[0] - key2[0]) <= 1 or abs(key1[1] - key2[1]) <= 1) and abs(key1[0] - key2[0]) + abs(key1[1] - key2[1]) < 2:
                is_neighbor = 1
            pj = cells[key2]/N
            denom_local_entropy += is_neighbor*pj
        loc_entropy -= pi*math.log2(denom_local_entropy)/math.log2(num_cells)
    return loc_entropy

def get_cohesion_radius(positions):
    center_of_mass = np.mean(positions, axis=0)
    return np.max(positions - center_of_mass)


def get_metrics(positions):
    positions = np.array(positions)
    dists = distance.cdist(positions, positions, 'euclidean')
    filter_non_neighbors = dists > max_distance
    filter_non_neighbors_prime = (dists > max_distance) | (dists == 0)
    filter_neighbors = (dists <= max_distance) & (dists > 0)
    adjacency_matrix = np.where(filter_non_neighbors, 0, dists)
    adjacency_matrix[dists == 0] = 0
    adjacency_matrix[filter_neighbors] = 1
    num_edges = np.sum(filter_neighbors)
    average_neighbor_distance = 0
    d = max_distance/lattice_scale
    if (num_edges > 0):
        average_neighbor_distance = np.sum(abs(np.where(filter_non_neighbors_prime, d, dists)-d))/num_edges
        #energy = np.sum(((np.where(filter_non_neighbors_prime, d, dists))*100-d*100)**2)/(num_edges + 1)
        #energy = energy/((100*d)**2)

    else:
        average_neighbor_distance = 0

    return adjacency_matrix, average_neighbor_distance

def get_relative_connectivity(adjacency):
    return 1/(float(adjacency.shape[0])-1)*np.linalg.matrix_rank(adjacency)

def get_degree_matrix(adjacency_matrix):
    return np.diag(np.sum(adjacency_matrix, axis=1))

def get_formation_change_rate(positions, old_positions, target_pos):
    return np.sum(positions - target_pos)**2/np.sum(old_positions - target_pos)**2

def get_algebraic_conn(laplacian):
    return np.sort(np.linalg.eig(laplacian)[0])[1]

def get_displacement(positions, old_positions):
    disp = positions - old_positions
    sum = 0
    for v_i in disp:
        sum += np.dot(100*v_i, 100*v_i.T)
    return sum/positions.shape[0]
    
def get_distance_from_target(positions):
    return np.mean(np.linalg.norm(positions, axis=1))



def evaluate(file_path):
    with open(file_path, 'r') as file:
        i = 1
        positions = []
        relative_connectivity = []
        avg_neighbor_distance = []
        old_positions = []
        v_mismatch = []
        target_dist = []

        for line in file:
            words = line.split()
            if int(words[0]) != i:
                adjacency_matrix, avg_n_distance = get_metrics(positions)
                degree_matrix = get_degree_matrix(adjacency_matrix)
                laplacian_matrix = degree_matrix - adjacency_matrix
                relative_connectivity.append(get_relative_connectivity(laplacian_matrix))
                avg_neighbor_distance.append(avg_n_distance)
                target_dist.append(get_distance_from_target(positions))
                positions = np.array(positions)
                
                if len(old_positions) != 0:
                    v_mismatch = np.append(v_mismatch, get_displacement(positions, old_positions))

                old_positions = positions
                positions = []
                obstacle_avoidance = []
                i+=1

            if (len(words) > 2):
                positions.append([float(words[1]), float(words[2])])

        return relative_connectivity, avg_neighbor_distance, v_mismatch, target_dist


def plot():
    
    fig, axes = plt.subplots(2, 4, figsize=(12, 8))
    x = range(1, len(relative_connectivity) + 1)
    sns.set(style="whitegrid")
    sns.lineplot(x=x, y=relative_connectivity, ax=axes[0, 0])
    axes[0,0].set_xlabel("Ticks")
    axes[0,0].set_ylabel(r'$\lambda_R(q)$')

    sns.lineplot(x=x, y=avg_neighbor_distance, ax=axes[0,2])
    axes[0,2].set_xlabel("Ticks")
    axes[0,2].set_ylabel(r'$\eta(q)$')

    sns.lineplot(x=np.arange(len(target_distance)), y=target_distance, ax=axes[1,2])
    axes[1,2].set_xlabel("Ticks")
    axes[1,2].set_ylabel(r'$\max ||q_i - q_\gamma||$')

    sns.lineplot(x=np.arange(len(v_mismatch)), y=v_mismatch, ax=axes[0,1])
    axes[0,1].set_xlabel("Ticks")
    axes[0,1].set_ylabel(r'$\kappa$')

    plt.tight_layout()
    # Save the entire figure with all subplots as an image in the current folder
    plot_name = "metrics_" + file_path + ".png"
    plt.savefig(plot_name, dpi=300)

    max_t = 0
    for i in range(len(relative_connectivity)):
        if relative_connectivity[i] < 1:
            max_t = i
    
    max_t2 = 0
    for z in range(max_t, len(v_mismatch)):
        if v_mismatch[z] > 0:
            max_t2 = z

    print(max_t2 - max_t)
    # Show the plots
    plt.show()

if __name__ == "__main__":
    file_path = sys.argv[1]
    relative_connectivity, avg_neighbor_distance, v_mismatch, target_distance = evaluate(sys.argv[1])
    plot()





