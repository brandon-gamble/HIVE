'''
TO DO
- change saturation from speed to individual motor omegas
    - self.omega_max_l and _r (from test data)
- things that will change with better test data:
    - controller gains
    - sys id variables of motor
        -> MAKE IT CLEAR WHERE THESE VALUES PLUG IN
- fix s_max calculation

'''

import path_functions as path_fn
import numpy as np
import matplotlib.patches as patches
from matplotlib import animation

class HIVE:
    def __init__(self,num_timesteps, startpos,
                    half_width, half_length, wheel_radius,
                    motor_l_params, motor_r_params):
        self.meter2pixel = 3779.52
        self.rpm2radsec = 0.10472

        #########################
        # motor characteristics #
        #########################
        # equation is form omega/u = k/(Js+b)
        self.J_l = motor_l_params[0]
        self.k_l = motor_l_params[1]
        self.b_l = motor_l_params[2]
        self.omega_max_l = motor_l_params[3] # [rad/s]
        self.omega_min_l = motor_l_params[4] # [rad/s]

        self.J_r = motor_r_params[0]
        self.k_r = motor_r_params[1]
        self.b_r = motor_r_params[2]
        self.omega_max_r = motor_r_params[3] # [rad/s]
        self.omega_min_r = motor_r_params[4] # [rad/s]

        ################################
        #  dims, pos/pose, velocities  #
        ################################
        # robot dims
        self.W = half_width # half_width
        self.L = half_length # half_length
        self.R = wheel_radius # wheel_radius

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

        # max speed by average max wheel omega
        self.s_max = (self.omega_max_l+self.omega_max_r)/2*self.R # [m/s]

        ################################
        #         initialize           #
        ################################
        self.x[0], self.y[0] = startpos
        self.target = np.zeros([2,1]) # target
        self.closest = np.zeros([2,1]) # closest point on path

        # to keep track of targets and closest point
        self.log_targ = np.zeros([2,num_timesteps])
        self.log_closest = np.zeros([2,num_timesteps])

        # keep track of control actuations
        self.log_u_l = np.zeros(num_timesteps)
        self.log_u_r = np.zeros(num_timesteps)

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
            if k % ratio == 0:
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
                s_des = min(np.linalg.norm(u),self.s_max)
                phi_des = np.arctan2(u[1],u[0])

                # error in heading
                error_phi = phi_des - self.phi_measured
                # need to use atan2 to unwind error
                error_phi = np.arctan2(np.sin(error_phi),np.cos(error_phi))

                # proportional control
                omega_des = error_phi*kp_heading

                # motor setpoints
                omega_l_des = (s_des - omega_des*self.W)/self.R
                omega_r_des = (s_des + omega_des*self.W)/self.R
                # saturation of motor setpoints
                omega_l_des = max(self.omega_min_l,min(self.omega_max_l,omega_l_des))
                omega_r_des = max(self.omega_min_r,min(self.omega_max_r,omega_r_des))

                # motor left control
                error_l = omega_l_des - self.omega_l_measured
                u_integral_l = u_integral_l + error_l*T_cont
                u_l = kp_l*error_l + ki_l*u_integral_l

                # motor right control
                error_r = omega_r_des - self.omega_r_measured
                u_integral_r = u_integral_r + error_r*T_cont
                u_r = kp_r*error_r + ki_r*u_integral_r

                # saturation of actuation
                u_l = max(-255,min(255,u_l))
                u_r = max(-255,min(255,u_r))


            # end of controller loop

            ###############
            #    PLANT    #
            ###############
            self.omega_l[k+1] = self.omega_l[k] + T_phys/self.J_l*(self.k_l*u_l-self.b_l*self.omega_l[k])
            self.omega_r[k+1] = self.omega_r[k] + T_phys/self.J_r*(self.k_r*u_r-self.b_r*self.omega_r[k])

            self.s[k+1] = (self.omega_l[k+1]+self.omega_r[k+1])*self.R/2
            self.omega[k+1] = (self.omega_r[k+1]-self.omega_l[k+1])*self.R/2/self.W
            self.phi[k+1] = self.phi[k] + self.omega[k]*T_phys
            self.x[k+1] = self.x[k] + self.s[k]*np.cos(self.phi[k])*T_phys
            self.y[k+1] = self.y[k] + self.s[k]*np.sin(self.phi[k])*T_phys

            self.log_targ[0,k] = self.target[0]
            self.log_targ[1,k] = self.target[1]

            self.log_closest[0,k] = self.closest[0]
            self.log_closest[1,k] = self.closest[1]

            self.log_u_l[k] = u_l
            self.log_u_r[k] = u_r

        return

def get_hive_vertices(half_length,half_width,x,y,phi):
    L = half_length
    W = half_width

    LW = np.array([[L, 0],[0, W]])
    signs = np.array([[0,1,1,-1,-1,1,1],[0,0,1,1,-1,-1,0]])

    # vertices of robot centered at origin with phi=0
    # vertices are center, front center, then the four corners (CCW),
    # then front center repeated again
    vertices = np.dot(LW,signs)

    # rotate by phi
    rotmat = np.array([[np.cos(phi),-np.sin(phi)],[np.sin(phi),np.cos(phi)]])
    vertices = np.dot(rotmat,vertices)

    # shift to xy
    vertices[0,:] = vertices[0,:] + x
    vertices[1,:] = vertices[1,:] + y

    return vertices

def animatePath(xy_array, phi, waypoints, obstacle_list, map_x, map_y,
                    frameDelay=1,
                    pt_skip = 10,
                    width = 2,
                    save=False,
                    ghost=True,
                    draw=True):
    '''
    input:	xy_array  [2xN]
            phi       [1xN]
            waypoints [2xN]

    output:	animation of drawing

    Xargs:
        width:  float   line width in plot
        save:   BOOL    true -> save mp4
        draw:   BOOL    true -> enable "pen" trace
        ghost:  BOOL    true -> show full path underlay
    '''

    numFrames = int(max(xy_array.shape)/pt_skip)

    fig = plt.figure()
    fig.set_dpi(100)
    fig.set_size_inches(7, 6.5)

    minx = map_x[0]
    maxx = map_x[1]

    miny = map_y[0]
    maxy = map_y[1]

    ax = plt.axes(xlim=(minx, maxx),
                    ylim=(miny, maxy))

    # vertices at t=0
    vertices = get_hive_vertices(my_hive.L,my_hive.W,
                                my_hive.x[0],my_hive.y[0],my_hive.phi[0])
    vertices = vertices.T
    # vertices = np.array([
    #     # [self.A[0],self.A[1]],
    #     # [B[0,0], B[0,1]],
    #     [xy_array[0,0], xy_array[1,0]]
    # ])

    patch = patches.Polygon(vertices, edgecolor=[100/255,0,1], linewidth=width, closed=False, fill=False)

    def init():
        ax.add_line(patch)

        if ghost == True:
            # plot full path, which will be drawn on top of
            plt.plot(xy_array[0,:],xy_array[1,:],linewidth=1)

        for obs in obstacle_list:
            self.plot_poly(obs,'r')
        # self.plot_workspace()

        return patch,

    def animate(i):
        vertices = get_hive_vertices(my_hive.L,my_hive.W,
                                    my_hive.x[i*pt_skip],my_hive.y[i*pt_skip],my_hive.phi[i*pt_skip])
        vertices=vertices.T
        # vertices = np.array([
        #     # self.A,
        #     # B[i],
        #     xy_array.T[i]
        # ])

        trace = xy_array.T[0:i*pt_skip]
        trace = trace[::-1]

        vertices = np.concatenate((vertices,trace))

        patch.set_xy(vertices)

        return patch,

    if draw == True:
        anim = animation.FuncAnimation(fig, animate,
                                        init_func = init,
                                        frames = numFrames,
                                        interval = frameDelay,
                                        blit = True)
    else:
        plt.plot(xy_array[0,:],xy_array[1,:])

    if save == True:
        # Set up formatting for the movie files
        # Writer = animation.writers['ffmpeg']
        # writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
        # writer = animation.FFMpegWriter(fps=15, metadata=dict(artist='Me'), bitrate=1800)
        # anim.save("movie.mp4", writer=writer)
        p = 0

    plt.show()

    return

if __name__ == "__main__":

    import matplotlib.pyplot as plt

    #   1   mimic HW4pathFollowing_tight_corner from autonomy
    #   2   dif path, tank dims
    #   3   tank dims and tank dif eq
    #   4   same as 3 but with updated tank dif eq and PID
    #   5   but with different controller timing (slowing to 20 ms)
    #       also spits out a pickle with control actuations
    #       also uses sys id 05 results
    test = 5

    if test == 1:

        # time step of physics and controller
        T_phys = 0.001
        T_cont = 0.01
        ratio = T_cont/T_phys

        # time vector for simulation
        max_time = 15
        t = np.arange(0,max_time,T_phys)
        n = len(t)

        start = [1,4]
        hW=0.1
        hL=0.1
        R=0.1
        my_hive = HIVE(num_timesteps=n, startpos=start,
                        half_width=hW, half_length=hL, wheel_radius=R,
                        motor_l_params=[0.1,2,1,999,-999],
                        motor_r_params=[0.1,2,1,999,-999])
        my_hive.s_max = 2

        waypoints = np.array([[2,2],[3,6], [7,7], [7,0], [-2,-2],[3,-2]]).T

        my_hive.simulate_follow(waypoints,
                                n,T_phys,T_cont,
                                kp_heading=2,
                                PI_left=[1/3, 10/3],
                                PI_right=[1/3, 10/3],
                                proj_dist=2)


        # plt.scatter(waypoints[0,:], waypoints[1,:])
        # plt.plot(my_hive.x,my_hive.y)

        # plot waypoints and path
        plt.subplot(2,2,1)
        plt.scatter(waypoints[0,0:-1],waypoints[1,0:-1],label='waypoints')
        plt.scatter(start[0],start[1],marker='x',label='start')
        plt.scatter(waypoints[0,-1],waypoints[1,-1],marker='x',label='finish')
        plt.plot(my_hive.x,my_hive.y,label='HIVE path')
        plt.title('HIVE path')
        plt.legend()

        # plot acutations
        plt.subplot(2,2,2)
        plt.plot(t,my_hive.log_u_l,label='u_l')
        plt.plot(t,my_hive.log_u_r,label='u_r')
        plt.title('actuator commands')
        plt.legend()

        # plot states x,y,phi thru time
        plt.subplot(2,2,3)
        plt.plot(t,my_hive.x,label='x')
        plt.plot(t,my_hive.y,label='y')
        plt.plot(t,my_hive.phi,label='phi')
        plt.title('positional states')
        plt.legend()

        # plot states omega and s thru time
        plt.subplot(2,2,4)
        plt.plot(t,my_hive.omega,label='omega')
        # plt.plot(t,my_hive.omega_l,label='omega_l')
        # plt.plot(t,my_hive.omega_r,label='omega_r')
        plt.plot(t,my_hive.s,label='s')
        plt.title('d/dt states')
        plt.legend()

        plt.show()

        my_xy = np.array([my_hive.x,my_hive.y])
        animatePath(xy_array=my_xy, phi=my_hive.phi,
                        waypoints=waypoints, obstacle_list=[],
                        map_x=[-4,8], map_y=[-4,8])
                        # pt_skip=10,
                        # frameDelay=1,
                        # width = 2,
                        # save=True,
                        # ghost=True,
                        # draw=True)


    elif test == 2:
        # time step of physics and controller
        T_phys = 0.001
        T_cont = 0.01
        ratio = T_cont/T_phys

        # time vector for simulation
        max_time = 30
        t = np.arange(0,max_time,T_phys)
        n = len(t)

        start = [0,0]
        hW=0.079
        hL=0.174
        R =0.022
        my_hive = HIVE(num_timesteps=n, startpos=start,
                        half_width=hW, half_length=hL, wheel_radius=R,
                        motor_l_params=[0.1,2,1,999,-999],
                        motor_r_params=[0.1,2,1,999,-999])

        my_hive.s_max = 1

        waypoints = np.array([[0.1,0.05],[0.2,0.05],[0.4,0.025],[0.6,0],[0.6,0.5],[0.5,0.6],[0.3,0.5],[0,0.2]]).T

        my_hive.simulate_follow(waypoints,
                                n,T_phys,T_cont,
                                kp_heading=2,
                                PI_left=[1/3, 10/3],
                                PI_right=[1/3, 10/3],
                                proj_dist=0.1)


        # plt.scatter(waypoints[0,:], waypoints[1,:])
        # plt.plot(my_hive.x,my_hive.y)

        # plot waypoints and path
        plt.subplot(2,2,1)
        plt.scatter(waypoints[0,0:-1],waypoints[1,0:-1],label='waypoints')
        plt.scatter(start[0],start[1],marker='x',label='start')
        plt.scatter(waypoints[0,-1],waypoints[1,-1],marker='x',label='finish')
        plt.plot(my_hive.x,my_hive.y,label='HIVE path')
        plt.title('HIVE path')
        plt.legend()

        # plot acutations
        plt.subplot(2,2,2)
        plt.plot(t,my_hive.log_u_l,label='u_l')
        plt.plot(t,my_hive.log_u_r,label='u_r')
        plt.title('actuator commands')
        plt.legend()

        # plot states x,y,phi thru time
        plt.subplot(2,2,3)
        plt.plot(t,my_hive.x,label='x')
        plt.plot(t,my_hive.y,label='y')
        plt.plot(t,my_hive.phi,label='phi')
        plt.title('positional states')
        plt.legend()

        # plot states omega and s thru time
        plt.subplot(2,2,4)
        plt.plot(t,my_hive.omega,label='omega')
        # plt.plot(t,my_hive.omega_l,label='omega_l')
        # plt.plot(t,my_hive.omega_r,label='omega_r')
        plt.plot(t,my_hive.s,label='s')
        plt.title('d/dt states')
        plt.legend()

        plt.show()

        my_xy = np.array([my_hive.x,my_hive.y])
        animatePath(xy_array=my_xy, phi=my_hive.phi,
                        waypoints=waypoints, obstacle_list=[],
                        map_x=[-.2,.8], map_y=[-.2,.8],
                        pt_skip=20)
                        # frameDelay=1,
                        # width = 2,
                        # save=True,
                        # ghost=True,
                        # draw=True)
    elif test == 3:
        # time step of physics and controller
        T_phys = 0.001
        T_cont = 0.01
        ratio = T_cont/T_phys

        # time vector for simulation
        max_time = 30
        t = np.arange(0,max_time,T_phys)
        n = len(t)

        start = [0,0]
        hW=0.079
        hL=0.174
        R =0.022
        my_hive = HIVE(num_timesteps=n, startpos=start,
                        half_width=hW, half_length=hL, wheel_radius=R,
                        motor_l_params=[1, 0.166, 13.35, 3, -2.5],
                        motor_r_params=[1, 0.1777,14.39, 3, -2.5])
        my_hive.s_max = 2

        waypoints = np.array([[0.1,0.05],[0.2,0.05],[0.4,0.025],[0.6,0],[0.6,0.5],[0.5,0.6],[0.3,0.5],[0,0.2]]).T

        my_hive.simulate_follow(waypoints,
                                n,T_phys,T_cont,
                                kp_heading=5,
                                PI_left=[420, 9620],
                                PI_right=[360, 9770],
                                proj_dist=0.1)



        # plot waypoints and path
        plt.subplot(2,2,1)
        plt.scatter(waypoints[0,0:-1],waypoints[1,0:-1],label='waypoints')
        plt.scatter(start[0],start[1],marker='x',label='start')
        plt.scatter(waypoints[0,-1],waypoints[1,-1],marker='x',label='finish')
        plt.plot(my_hive.x,my_hive.y,label='HIVE path')
        plt.title('HIVE path')
        plt.legend()

        # plot acutations
        plt.subplot(2,2,2)
        plt.plot(t,my_hive.log_u_l,label='u_l')
        plt.plot(t,my_hive.log_u_r,label='u_r')
        plt.title('actuator commands')
        plt.legend()

        # plot states x,y,phi thru time
        plt.subplot(2,2,3)
        plt.plot(t,my_hive.x,label='x')
        plt.plot(t,my_hive.y,label='y')
        plt.plot(t,my_hive.phi,label='phi')
        plt.title('positional states')
        plt.legend()

        # plot states omega and s thru time
        plt.subplot(2,2,4)
        plt.plot(t,my_hive.omega,label='omega')
        # plt.plot(t,my_hive.omega_l,label='omega_l')
        # plt.plot(t,my_hive.omega_r,label='omega_r')
        plt.plot(t,my_hive.s,label='s')
        plt.title('d/dt states')
        plt.legend()

        plt.show()

        my_xy = np.array([my_hive.x,my_hive.y])
        animatePath(xy_array=my_xy, phi=my_hive.phi,
                        waypoints=waypoints, obstacle_list=[],
                        map_x=[-.2,.8], map_y=[-.2,.8],
                        pt_skip=20)
                        # frameDelay=1,
                        # width = 2,
                        # save=True,
                        # ghost=True,
                        # draw=True)
    elif test == 4:
        # time step of physics and controller
        T_phys = 0.001
        T_cont = 0.01
        ratio = T_cont/T_phys

        # time vector for simulation
        max_time = 30
        t = np.arange(0,max_time,T_phys)
        n = len(t)

        start = [0,0]
        hW=0.079
        hL=0.174
        R =0.022
        my_hive = HIVE(num_timesteps=n, startpos=start,
                        half_width=hW, half_length=hL, wheel_radius=R,
                        motor_l_params=[1, 2.411, 19.81, 30, -25],
                        motor_r_params=[1, 2.431, 20.10, 30, -25])
        my_hive.s_max = 2

        waypoints = np.array([[0.1,0.05],[0.2,0.05],[0.4,0.025],[0.6,0],[0.6,0.5],[0.5,0.6],[0.3,0.5],[0,0.2]]).T

        my_hive.simulate_follow(waypoints,
                                n,T_phys,T_cont,
                                kp_heading=5,
                                PI_left=[26.4, 811],
                                PI_right=[27.0,771],
                                proj_dist=0.1)
        # PI_left has:
        #   t_rise    = 0.0263 s
        #   t_settle  = 0.1020 s
        #   overshoot = 4.23   %
        # PI_right has:
        #   t_rise    = 0.0269 s
        #   t_settle  = 0.0962 s
        #   overshoot = 3.20   %

        # plot waypoints and path
        plt.subplot(2,2,1)
        plt.scatter(waypoints[0,0:-1],waypoints[1,0:-1],label='waypoints')
        plt.scatter(start[0],start[1],marker='x',label='start')
        plt.scatter(waypoints[0,-1],waypoints[1,-1],marker='x',label='finish')
        plt.plot(my_hive.x,my_hive.y,label='HIVE path')
        plt.title('HIVE path')
        plt.legend()

        # plot acutations
        plt.subplot(2,2,2)
        plt.plot(t,my_hive.log_u_l,label='u_l')
        plt.plot(t,my_hive.log_u_r,label='u_r')
        plt.title('actuator commands')
        plt.legend()

        # plot states x,y,phi thru time
        plt.subplot(2,2,3)
        plt.plot(t,my_hive.x,label='x')
        plt.plot(t,my_hive.y,label='y')
        plt.plot(t,my_hive.phi,label='phi')
        plt.title('positional states')
        plt.legend()

        # plot states omega and s thru time
        plt.subplot(2,2,4)
        plt.plot(t,my_hive.omega,label='omega')
        # plt.plot(t,my_hive.omega_l,label='omega_l')
        # plt.plot(t,my_hive.omega_r,label='omega_r')
        plt.plot(t,my_hive.s,label='s')
        plt.title('d/dt states')
        plt.legend()

        plt.show()

        my_xy = np.array([my_hive.x,my_hive.y])
        animatePath(xy_array=my_xy, phi=my_hive.phi,
                        waypoints=waypoints, obstacle_list=[],
                        map_x=[-.2,.8], map_y=[-.2,.8],
                        pt_skip=20)
                        # frameDelay=1,
                        # width = 2,
                        # save=True,
                        # ghost=True,
                        # draw=True)
    elif test == 5:
        # time step of physics and controller
        T_phys = 0.001
        T_cont = 0.02
        ratio = T_cont/T_phys

        # time vector for simulation
        max_time = 30
        t = np.arange(0,max_time,T_phys)
        n = len(t)

        start = [0,0]
        hW=0.079
        hL=0.174
        R =0.022
        my_hive = HIVE(num_timesteps=n, startpos=start,
                        half_width=hW, half_length=hL, wheel_radius=R,
                        motor_l_params=[1, 1.809, 16.17, 30, -25],
                        motor_r_params=[1, 2.048, 18.15, 30, -25])
        my_hive.s_max = 2

        waypoints = np.array([[0.1,0.05],[0.2,0.05],[0.4,0.025],[0.6,0],[0.6,0.5],[0.5,0.6],[0.3,0.5],[0,0.2]]).T

        my_hive.simulate_follow(waypoints,
                                n,T_phys,T_cont,
                                kp_heading=5,
                                PI_left=[26.4, 811],
                                PI_right=[27.0,771],
                                proj_dist=0.1)
        # PI_left has:
        #   t_rise    = 0.0263 s
        #   t_settle  = 0.1020 s
        #   overshoot = 4.23   %
        # PI_right has:
        #   t_rise    = 0.0269 s
        #   t_settle  = 0.0962 s
        #   overshoot = 3.20   %

        # plot waypoints and path
        plt.subplot(2,2,1)
        plt.scatter(waypoints[0,0:-1],waypoints[1,0:-1],label='waypoints')
        plt.scatter(start[0],start[1],marker='x',label='start')
        plt.scatter(waypoints[0,-1],waypoints[1,-1],marker='x',label='finish')
        plt.plot(my_hive.x,my_hive.y,label='HIVE path')
        plt.title('HIVE path')
        plt.legend()

        # plot acutations
        plt.subplot(2,2,2)
        plt.plot(t,my_hive.log_u_l,label='u_l')
        plt.plot(t,my_hive.log_u_r,label='u_r')
        plt.title('actuator commands')
        plt.legend()

        # plot states x,y,phi thru time
        plt.subplot(2,2,3)
        plt.plot(t,my_hive.x,label='x')
        plt.plot(t,my_hive.y,label='y')
        plt.plot(t,my_hive.phi,label='phi')
        plt.title('positional states')
        plt.legend()

        # plot states omega and s thru time
        plt.subplot(2,2,4)
        plt.plot(t,my_hive.omega,label='omega')
        # plt.plot(t,my_hive.omega_l,label='omega_l')
        # plt.plot(t,my_hive.omega_r,label='omega_r')
        plt.plot(t,my_hive.s,label='s')
        plt.title('d/dt states')
        plt.legend()

        plt.show()

        my_xy = np.array([my_hive.x,my_hive.y])
        animatePath(xy_array=my_xy, phi=my_hive.phi,
                        waypoints=waypoints, obstacle_list=[],
                        map_x=[-.2,.8], map_y=[-.2,.8],
                        pt_skip=20)
                        # frameDelay=1,
                        # width = 2,
                        # save=True,
                        # ghost=True,
                        # draw=True)


        ##########################
        #     SAVE TEST DATA     #
        ##########################
        # generate filename string
        if test < 10:
            num = '0' + str(test)
        else:
            num = str(test)
        filename = 'follow_path_test' + num + '_actuatorCommands.txt'

        # build array of actuator data
        # shape nx2 (two tall columns)
        actuation_data = np.array([my_hive.log_u_l,my_hive.log_u_r]).T

        # save file
        np.savetxt(filename, actuation_data, fmt='%d')
