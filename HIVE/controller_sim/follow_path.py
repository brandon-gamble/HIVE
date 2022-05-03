'''
TO DO
- change saturation from speed to individual motor omegas
    - self.omega_max_l and _r (from test data)
- things that will change with better test data:
    - controller gains
    - sys id variables of motor
        -> MAKE IT CLEAR WHERE THESE VALUES PLUG IN


'''

import path_functions as path_fn
import numpy as np

class HIVE:
    def __init__(self,num_timesteps,L,R,startpos):
        self.meter2pixel = 3779.52
        self.rpm2radsec = 0.10472

        #########################
        # motor characteristics #
        #########################
        self.max_speed = 2 # [m/s]

        self.J_l = 0.1
        self.k_l = 2
        self.b_l = 1
        self.omega_max_l = []

        self.J_r = 0.1
        self.k_r = 2
        self.b_r = 1
        self.omega_max_r = []

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

    def simulate_follow(self,waypoints,
                            n_steps,T_phys,T_cont,
                            kp_heading,PI_left,PI_right,
                            proj_dist):
        ratio = T_cont/T_phys

        # proportional and integral gains for left motor
        kp_l = PI_left[0]
        ki_l = PI_left[1]

        # proportional and integral gains for right motor
        kp_r = PI_right[0]
        ki_r = PI_right[1]

        # heading controller is simple propotional with kp_heading

        # initialize motor control inputs at 0
        u_integral_l = 0
        u_integral_r = 0

        # initialize control actuations at 0
        u_l = 0
        u_r = 0

        for k in range(n_steps-1):
            ##############
            # CONTROLLER #
            ##############
            # evaluate controller every time k is a multiple of the
            # ratio of time scales of physics and controller
            if (k+1) % ratio == 0:
                #######################################
                # pretend to take sensor measurements #
                #######################################
                self.x_measured = self.x[k]
                self.y_measured = self.y[k]
                self.phi_measured = self.phi[k]
                self.omega_l_measured = self.omega_l[k]
                self.omega_r_measured = self.omega_r[k]

                # find target
                P = [self.x_measured, self.y_measured]
                self.target, self.closest = path_fn.find_target(P,waypoints,proj_dist)
                targ_x = self.target[0]
                targ_y = self.target[1]

                # velocity vector and desired speed/heading
                u = np.array([[targ_x - self.x_measured],[targ_y - self.y_measured]])
                s_des = min(np.linalg.norm(u),self.max_speed)
                phi_des = np.arctan2(u[1],u[0])

                # error in heading
                error_phi = phi_des - self.phi_measured
                # need to use atan2 to unwind error
                error_phi = np.arctan2(np.sin(error_phi),np.cos(error_phi))

                # proportional control
                omega_des = error_phi*kp_heading

                # motor setpoints
                omega_r_des = (s_des + omega_des*self.L)/self.R
                omega_l_des = (s_des - omega_des*self.L)/self.R

                # motor left control
                error_l = omega_l_des - self.omega_l_measured
                u_integral_l = u_integral_l + error_l*T_cont
                u_l = kp_l*error_l + ki_l*u_integral_l

                # motor right control
                error_r = omega_r_des - self.omega_r_measured
                u_integral_r = u_integral_r + error_r*T_cont
                u_r = kp_r*error_r + ki_r*u_integral_r


            # end of controller loop

            ###############
            #    PLANT    #
            ###############
            self.omega_l[k+1] = self.omega_l[k] + T_phys/self.J_l*(self.k_l*u_l-self.b_l*self.omega_l[k])
            self.omega_r[k+1] = self.omega_r[k] + T_phys/self.J_r*(self.k_r*u_r-self.b_r*self.omega_r[k])

            self.s[k+1] = (self.omega_l[k+1]+self.omega_r[k+1])*self.R/2
            self.omega[k+1] = (self.omega_r[k+1]-self.omega_l[k+1])*self.R/2/self.L
            self.phi[k+1] = self.phi[k] + self.omega[k]*T_phys
            self.x[k+1] = self.x[k] + self.s[k]*np.cos(self.phi[k])*T_phys
            self.y[k+1] = self.y[k] + self.s[k]*np.sin(self.phi[k])*T_phys

            self.log_targ[0,k] = self.target[0]
            self.log_targ[1,k] = self.target[1]

            self.log_closest[0,k] = self.closest[0]
            self.log_closest[1,k] = self.closest[1]

        return

if __name__ == "__main__":

    import matplotlib.pyplot as plt

    #   1   mimic HW4pathFollowing_tight_corner from autonomy
    #   2
    test = 1

    if test == 1:

        # time step of physics and controller
        T_phys = 0.001
        T_cont = 0.01
        ratio = T_cont/T_phys

        # time vector for simulation
        max_time = 15
        t = np.arange(0,max_time,T_phys)
        n = len(t)

        my_hive = HIVE(n,0.1,0.1,[1,4])

        waypoints = np.array([[2,2], [3,6], [7,7], [7,0], [-2,-2],[3,-2]]).T

        my_hive.simulate_follow(waypoints,
                                n,T_phys,T_cont,
                                kp_heading=2,
                                PI_left=[1/3, 10/3],
                                PI_right=[1/3, 10/3],
                                proj_dist=2)

        # plt.scatter(waypoints[0,:], waypoints[1,:])
        # plt.plot(my_hive.x,my_hive.y)

        # plot waypoints and path
        plt.subplot(1,3,1)
        plt.scatter(waypoints[0,:],waypoints[1,:],label='waypoints')
        plt.plot(my_hive.x,my_hive.y,label='HIVE path')
        plt.title('HIVE path')
        plt.legend()

        # plot states x,y,phi thru time
        plt.subplot(1,3,2)
        plt.plot(t,my_hive.x,label='x')
        plt.plot(t,my_hive.y,label='y')
        plt.plot(t,my_hive.phi,label='phi')
        plt.legend()

        # plot states omega and s thru time
        plt.subplot(1,3,3)
        plt.plot(t,my_hive.omega,label='omega')
        # plt.plot(t,my_hive.omega_l,label='omega_l')
        # plt.plot(t,my_hive.omega_r,label='omega_r')
        plt.plot(t,my_hive.s,label='s')
        plt.legend()

        plt.show()




    elif test == 2:
        a = []

    # elif test == 3:
    # elif test == 4:
    # elif test == 5:
