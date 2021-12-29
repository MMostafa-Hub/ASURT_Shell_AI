import numpy as np

alpha = 1.0
beta = 1.0

# graph_points = np.array(
#     [[0.,    0.],
#      [114.28714842,   41.98759603],
#      [62.47783741,  -10.16037304],
#      [24.5058101,  179.34217451],
#      [-9.51912637,   86.66461683]
#      ]
# )
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


def get_max_prob_index(root, visited, beta=1, alpha=1):
    return np.argmax(np.multiply(np.power(pheremone_matrix[root], alpha), np.power(np.logical_xor(visited, 1), beta)))


def dist(point_a, point_b):
    return np.linalg.norm(point_a - point_b)


def update_pheremones(distances_travelled, paths, Q=100, row=.5):
    # k ants
    for k in range(len(paths)):
        for i in range(len(paths[k]) - 1):
            node_i = paths[k][i]
            after_node_i = paths[k][i+1]

            # as the graph is undirceted
            pheremone_matrix[node_i][after_node_i] = row * \
                pheremone_matrix[node_i][after_node_i] + \
                Q/distances_travelled[k]
            pheremone_matrix[after_node_i][node_i] = row * \
                pheremone_matrix[after_node_i][node_i] + \
                Q/distances_travelled[k]


def ACO():
    n_iterations = 50
    k_ants = 50

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
            # path_nodes = np.array([root])
            path_indices = [root_index]

            iterator_root = root
            iterator_root_index = root_index
            while not np.all(visited == True):

                # if it's the first iteration
                if get_max_prob_index(iterator_root_index, visited) == 0:
                    # a random root choosen from neighbour_nodes
                    # which is garanteed to be not visited
                    new_root = np.random.default_rng().choice(neighbour_nodes, 1, replace=False)

                    # the indexing is based on graph_points
                    new_root_index = np.where(
                        np.all(graph_points == new_root, axis=1))[0][0]

                # if it's not the first iteration
                else:
                    # making the next root with maximum prob in the probability matrix
                    new_root_index = get_max_prob_index(
                        iterator_root_index, visited)
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
                # path_nodes = np.vstack([path_nodes, new_root])
                path_indices.append(new_root_index)

                # distance travelled by the ant
                distance = dist(iterator_root, new_root)
                distance_travelled += distance
                # print(
                #     f"the distance from {iterator_root_index} to {new_root_index} is {distance}")

                # Iterating
                iterator_root = new_root
                iterator_root_index = new_root_index

            # appending the index of the root node to make a cycle
            path_indices.append(root_index)

            # adding the distance from the last node to the root
            distance_travelled += dist(iterator_root, root)

            # Update the paths
            paths.append(path_indices)

            # update the distances travelled
            distances_travelled.append(distance_travelled)
            # print(distances_travelled)
        print(paths)
        print(distances_travelled)

        # update the pheremone Matrix
        update_pheremones(distances_travelled, paths)


ACO()
print(pheremone_matrix)
