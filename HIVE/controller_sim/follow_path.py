import path_functions as path_fn
import numpy as np

class HIVE:
    def __init__(self,num_timesteps,L,R,startpos):
        self.meter2pixel = 3779.52
        self.rpm2radsec = 0.10472

        ################################
        #  dims, pos/pose, velocities  #
        ################################
        # robot dims
        self.L = L # half_width
        self.R = R # wheel_radius

        # position/orientation: x,y,heading
        self.x = np.zeros(num_timesteps)
        self.y = np.zeros(num_timesteps)
        self.phi = np.zeros(num_timesteps)

        # velocities
            # wheel rotational speeds
        self.omega_l = np.zeros(num_timesteps)
        self.omega_r = np.zeros(num_timesteps)
            # total robot rotation speed
        self.omega = np.zeros(num_timesteps)
            # total robot speed
        self.s = np.zeros(num_timesteps)

        # rpm saturation
        self.max_omega = 20
        self.min_omega = -15

        ################################
        #         initialize           #
        ################################
        self.x[0], self.y[0] = startpos
        self.target = np.zeros([2,1]) # target
        self.closest = np.zeros([2,1]) # closest point on path

        # to keep track of targets and closest point
        self.log_targ = np.zeros([2,num_timesteps])
        self.log_closest = np.zeros([2,num_timesteps])

        # sensors
        self.x_measured = 0
        self.y_measured = 0
        self.phi_measured = 0
        self.omega_l_measured = 0
        self.omega_r_measured = 0

    def XXget_normal_point(self,P,A,B):
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

    def XXfind_target(self,point,path,proj_dist):
        N_pts = max(path.shape)
        N_seg = N_pts - 1

        normal_vecs = np.zeros([2,N_seg])     # normal vec to each line segment in path
        normal_norms = np.zeros([N_seg])    # length of normal vecs^

        for k in range(N_seg):
            normal_vecs[:,k] = self.XXget_normal_point(point,path[:,k],path[:,k+1]) - point
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

    def simulate_follow(self,T_phys,T_cont,path):
        return



if __name__ == "__main__":

    # time step of physics and controller
    T_phys = 0.001
    T_cont = 0.01
    ratio = T_cont/T_phys

    # time vector for simulation
    t = np.arange(0,15,T_phys)
    n = len(t)

    my_hive = HIVE(n,0.1,0.1,[0,0])

    # 1 get normal point
    # 2 find target
    test = 2
    if test == 1:
        # path = np.asarray([[2,2],[3,6],[7,7],[7,0],[-2,-2],[3,-2]])
        path = np.array([[1,4],[-10,4]])
        path = path.T

        P = np.array([2,2])
        A = np.array([-1,1])
        B = np.array([3,1])
        print(path_fn.get_normal_point(P,A,B))

        P = np.array([4,2])
        print(path_fn.get_normal_point(P,A,B))

        P = np.array([2,2])
        A = np.array([-1,1])
        B = np.array([5,-5])
        print(path_fn.get_normal_point(P,A,B))

    elif test == 2:
        points =     np.array([[4,2], [3,8],   [10,5],    [11,1]]).T
        expected_T = np.array([[2,3], [5,5.8], [8.3,2.5], [10,0]]).T
        expected_C = np.array([[2,2], [4,5.5], [7.5,3.3], [9.5,0.5],]).T
        path = np.array([[0,0],[2,2],[2,5],[6,6],[10,0]])
        path = path.T
        proj_dist = 1

        for k in range(max(points.shape)):
            point = points[:,k]
            t,c = path_fn.find_target(point,path,proj_dist)
            # t,c = my_hive.XXfind_target(point,path,proj_dist)
            print("P: %.1f,%.1f \t T: %.1f,%.1f \t eT: %.1f,%.1f \t C: %.1f,%.1f \t eC: %.1f,%.1f" %
                (point[0],point[1],
                t[0],t[1],expected_T[0,k],expected_T[1,k],
                c[0],c[1],expected_C[0,k],expected_C[1,k]))

    # elif test == 3:
    # elif test == 4:
    # elif test == 5:
