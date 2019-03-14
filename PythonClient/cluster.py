from copy import deepcopy
import numpy as np
import random

def dist(A, B):
    # print(A,B)
    distances = []
    if len(A) != len(B) and len(A) == 1:
        A = [A[0]] * len(B)
    for i,b in enumerate(B):
        a = A[i]
        distance = abs(a[0] - b[0]) + abs(a[1] - b[1])
        distances.append(distance)
        distance = 0
    return distances

def min_index(distances,clusters,cluster_size, surplus):
    minimum = 1000
    for index in range(len(distances)):
        if distances[index] < minimum and (len(clusters[index]) < cluster_size or surplus):
            minimum_index = index
            minimum = distances[index]
    return minimum_index

def kmeans(bombs,k):
    ### Random ###
    # top_left = min([i[0] for i in bombs]),(min([i[1] for i in bombs]))
    # bottom_right = max([i[0] for i in bombs]),(max([i[1] for i in bombs]))
    # for i in range(k):
    #     random_position = random.randint(top_left[0],bottom_right[0]),random.randint(top_left[1],bottom_right[1])
    #     cluster_centers.append(random_position)

    deleted_bombs = []
    has_single_cluster = True
    bombs_number = len(bombs)
    cluster_size = bombs_number // k
    cluster_centers = [bombs[cluster_size * i] for i in range(k)]
    has_single_cluster = False
    error = 1
    while (error):
        clusters = [[] for i in range(k)]
        for index,bomb in enumerate(bombs):
            distances = dist([bomb], cluster_centers)
            if index + 1 > cluster_size * k:
                surplus = True
            else:
                surplus = False
            cluster_index = min_index(distances, clusters, cluster_size, surplus)
            clusters[cluster_index].append(bomb)
        cluster_centers_old = deepcopy(cluster_centers)
        for i in range(k):
            if clusters[i]:
                cluster_centers[i] = np.mean(clusters[i], axis=0)
        error = sum(dist(cluster_centers, cluster_centers_old))
    integer_centers = []
    for center in cluster_centers:
        integer_centers.append((int(center[0]),int(center[1])))
    return clusters, integer_centers
