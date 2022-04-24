import numpy as np

def get_normal_point(P,A,B):
    # find point on line segment closest to a given point
    # P is point
    # A is starting point of segment
    # B is end point of segment

    a = P - A
    b = B - A
    c = P - B

    normal_point = A + np.dot(a,b)/np.linalg.norm(b)**2*b;

    x_between_AB = (normal_point[0]>=min(A[0],B[0]) and normal_point[0]<=max(A[0],B[0]))
    y_between_AB = (normal_point[1]>=min(A[1],B[1]) and normal_point[1]<=max(A[1],B[1]))

    if x_between_AB and y_between_AB:
        return normal_point
    else:
        norm_to_A = np.linalg.norm(a)
        norm_to_B = np.linalg.norm(c)
        if norm_to_A < norm_to_B:
            normal_point = A;
        else:
            normal_point = B;

    return normal_point

def find_target(point,path,proj_dist):
    N_pts = max(path.shape)
    N_seg = N_pts - 1

    normal_vecs = np.zeros([2,N_seg])     # normal vec to each line segment in path
    normal_norms = np.zeros([N_seg])    # length of normal vecs^

    for k in range(N_seg):
        normal_vecs[:,k] = get_normal_point(point,path[:,k],path[:,k+1]) - point
        normal_norms[k] = np.linalg.norm(normal_vecs[:,k])

    index = np.argmin(normal_norms)     # index of smallest norm
    Q = normal_vecs[:,index]            # smallest normal vec
    closest = point + Q                 # closest point on path
    R = path[:,index+1]-closest         # distance from C to end of seg [index]
    norm_R = np.linalg.norm(R)          # get norm of R
    unit_R = R/norm_R                   # get unit vec

    if norm_R >= proj_dist:
        target = closest + unit_R*proj_dist
    else:
        proj_remain = proj_dist - norm_R
        seg = index+1
        while proj_remain > 0:
            if seg == N_seg:                # if looking past last segment, terminate at endpoint
                target = path[:,N_seg]
                return target, closest
            else:
                this_seg = path[:,seg+1] - path[:,seg]
                this_norm = np.linalg.norm(this_seg)
                unit_vec = this_seg/this_norm
                if proj_remain < this_norm: # project remaining along seg
                    this_proj = unit_vec*proj_remain
                    proj_remain = 0
                    target = path[:,seg] + this_proj
                else: # project fully along seg and go to next seg
                    proj_remain = proj_remain - this_norm


    return target, closest
