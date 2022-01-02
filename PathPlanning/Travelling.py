import numpy as np
import matplotlib.pyplot as plt
from numpy.core.fromnumeric import argmax

graph_points = np.array(
    [[0.,    0.],
     [114.28714842,   41.98759603],
     [62.47783741,  -10.16037304],
     [24.5058101,  179.34217451],
     [-9.51912637,   86.66461683],
     [-107.49794568,   23.53735607],
     [91.98201947,  153.39087436],
     [130.74459844,   95.76204485],
     [100.11681872,   -9.67705624],
     [-97.97803427, -116.85725875],
     [12.76266999,  200.86528354],
     [35.42322761,  -86.24626489],
     [-15.17293957,  -39.18455774],
     [50.71204359,  -43.96724648],
     [-165.73686143,  -40.45509785],
     [-59.11389879,  -75.68251789],
     [-65.86307249,  -51.20886232],
     [-118.65056562,   58.30787596],
     [0.25090536,   49.73685338],
     [95.02087717,   63.66352072]]
)

n_nodes = graph_points.shape[0]
prob_matrix = np.zeros((n_nodes, n_nodes))
pheremone_matrix = np.zeros((n_nodes, n_nodes))
distance_matrix = np.zeros((n_nodes, n_nodes))


def fill_distance_matrix():
    # Just fills the cost matrix between each node

    for i in range(n_nodes):
        for j in range(n_nodes):
            distance_matrix[i][j] = np.linalg.norm(
                graph_points[i] - graph_points[j])


def dist(point_a, point_b):
    # returns the Euclidian distance between two points
    return np.linalg.norm(point_a - point_b)


def get_max_prob_index(root_index, visited, beta=2, alpha=1):

    # list contains the probability of each unvisited node
    probs = []

    # we want the pheremones of the nodes we'didnt visit yet
    root_pheremones = np.multiply(
        pheremone_matrix[root_index], np.logical_xor(visited, 1))

    # if all the pheremones are zeros,it means that we are still in the first eteration
    if np.all(root_pheremones == 0):
        return -1

    # pher ** alpha
    pher_pow_alpha = np.power(root_pheremones, alpha)

    # ( 1 / dist ) ** beta
    dists_pow_beta = np.power(np.divide(1, distance_matrix[root_index]), beta)

    # since the dist can be zero so 1/dist can be inf , so we replace it with 0
    dists_pow_beta[dists_pow_beta == np.inf] = 0

    # sigma of all unvisited nodes = (pher_i ** alpha) * ((1 / dists_i) ** beta)
    s = np.sum(np.multiply(pher_pow_alpha, dists_pow_beta))

    # filling the probability list
    for pher, dist in zip(pher_pow_alpha, dists_pow_beta):
        probs.append((pher * dist) / s)

    # retunring the index of the max probability
    return probs.index(max(probs))


def update_pheremones(distances_travelled, paths, Q=100, row=.9):
    # k ants
    for k in range(len(paths)):
        for i in range(len(paths[k]) - 1):
            node_i = paths[k][i]
            after_node_i = paths[k][i+1]

            # pher_i_j = Q / distance travelled_k
            # as the graph is undirceted
            pheremone_matrix[node_i][after_node_i] = row * \
                pheremone_matrix[node_i][after_node_i] + \
                Q/distances_travelled[k]
            pheremone_matrix[after_node_i][node_i] = row * \
                pheremone_matrix[after_node_i][node_i] + \
                Q/distances_travelled[k]


def ACO():
    fill_distance_matrix()
    
    n_iterations = n_nodes + 1
    k_ants = n_nodes
    for i in range(n_iterations):

        # to store the total pherimones secreted by k_ants
        distances_travelled = []
        paths = []
        for k in range(k_ants):
            # Evaluates int array of Falses
            visited = np.zeros(n_nodes)

            # Distance travelled by the kth_ant
            distance_travelled = 0

            # a random root choosen from the graph
            root = np.random.default_rng().choice(graph_points, 1, replace=False)
            root_index = np.where(np.all(graph_points == root, axis=1))[0][0]

            if i == n_iterations-1:
                root = np.array([0, 0])
                root_index = 0

            # The root is marked as visited
            visited[root_index] = True

            # ndArray of all the nodes excpet the root
            neighbour_nodes = np.delete(graph_points, root_index, 0)

            # add the root node to the path
            path_indices = [root_index]

            iterator_root = root
            iterator_root_index = root_index
            while not np.all(visited == True):

                max_prob_index = get_max_prob_index(
                    iterator_root_index, visited)
                # if it's the first iteration
                if max_prob_index == -1:
                    # a random root choosen from neighbour_nodes
                    # which is garanteed to be not visited
                    new_root = np.random.default_rng().choice(neighbour_nodes, 1, replace=False)

                    # the indexing is based on graph_points
                    new_root_index = np.where(
                        np.all(graph_points == new_root, axis=1))[0][0]

                # if it's not the first iteration
                else:
                    # making the next root with maximum prob in the probability list
                    new_root_index = max_prob_index
                    new_root = graph_points[new_root_index]

                # The new_root is marked as visited
                visited[new_root_index] = True

                # taking the index of the root based on neighbours array
                new_root_index_neighbour = np.where(
                    np.all(neighbour_nodes == new_root, axis=1))[0][0]

                # ndArray of all the nodes excpet the new_root
                neighbour_nodes = np.delete(
                    neighbour_nodes, new_root_index_neighbour, 0)

                # add the new_root node to the path
                path_indices.append(new_root_index)

                # distance travelled by the ant
                distance_travelled += distance_matrix[iterator_root_index][new_root_index]

                # Iterating
                iterator_root = new_root
                iterator_root_index = new_root_index

            # appending the index of the root node to make a cycle
            path_indices.append(root_index)

            # adding the distance from the last node to the root
            distance_travelled += distance_matrix[iterator_root_index][root_index]

            # Update the paths
            paths.append(path_indices)

            # update the distances travelled
            distances_travelled.append(distance_travelled)
            # print(distances_travelled)
        print(paths)
        print(distances_travelled)

        # update the pheremone Matrix
        update_pheremones(distances_travelled, paths)
        print(pheremone_matrix)
    return paths[0]


# scatter plot of the graph points
plt.scatter(graph_points[:, 0], graph_points[:, 1])

# the best path from point 0,0 to point 0,0
plan = np.array([graph_points[i] for i in ACO()])
plt.plot(plan[:, 0], plan[:, 1])

plt.show()
