import pygame
import math

class Envir:
    def __init__(self, dimensions):
        # standard colors
        self.black = (0,0,0)
        self.white = (255,255,255)
        self.red = (255,0,0)
        self.green = (0,255,0)
        self.blue = (0,0,255)
        self.yel = (255,255,0)

        # map dimensions
        self.height, self.width = dimensions

        # window settings
        pygame.display.set_caption('Differential Drive Fleet Following')
        self.map=pygame.display.set_mode((self.width,self.height))

        # text vars
        self.font = pygame.font.Font('freesansbold.ttf',25)
        self.text = self.font.render('default',True,self.white,self.black)
        self.textRect = self.text.get_rect()
        self.textRect.center=(75,25) # x,y

    def write_lr_info(self, Vl, Vr, theta):
        txt = f'Vl = {Vl} Vr = {Vr} theta = {int(math.degrees(theta))}'
        self.text = self.font.render(txt,True,self.white,self.black)
        self.map.blit(self.text,self.textRect)

    def write_uomega_info(self, u, omega, theta):
        txt = f'u = {u} omega = {omega:.5f} theta = {int(math.degrees(theta))}'
        self.text = self.font.render(txt,True,self.white,self.black)
        self.map.blit(self.text,self.textRect)

    def draw_ref_frame(self,pos,rotation):
        n = 50

        center_x,center_y = pos
        x_axis = (center_x + n*math.cos(-rotation),
                    center_y + n*math.sin(-rotation))
        y_axis = (center_x + n*math.cos(-rotation+math.pi/2),
                    center_y + n*math.sin(-rotation+math.pi/2))

        pygame.draw.line(self.map,self.red,(center_x,center_y),x_axis,3)
        pygame.draw.line(self.map,self.green,(center_x,center_y),y_axis,3)


class Robot:
    def __init__(self, startpos, robotImg, width, follow=None):
        self.meter2pixel=3779.52

        self.leader = False # flag leader or not
        self.follow = follow # which bot to follow


        # robot dims
        self.x,self.y =startpos
        self.theta=0
        self.trail_set = []
        self.a = 20 # dist from wheels to center
        self.width = width
        self.u = 30 # pixel/s
        self.omega = 0
#         self.maxspeed = +0.02*self.meter2pixel
#         self.minspeed = -0.02*self.meter2pixel

        # skin
        self.img=pygame.image.load(robotImg)
        # every time we rotate image we deteriorate the image.
        # as such, we will hold the original image in self.img and use self.rotated
        # to always be only a single rotation command on self.img. this way we
        # do not accumulate deterioration of the image
        self.rotated=self.img
        self.rect=self.rotated.get_rect(center=(self.x,self.y))


    def draw_robot(self,map):
        map.blit(self.rotated,self.rect)


    def move(self, event=None):
        self.x = self.x + (self.u*math.cos(self.theta) - self.a*math.sin(self.theta)*self.omega)*dt
        self.y = self.y + (self.u*math.sin(self.theta) + self.a*math.cos(self.theta)*self.omega)*dt
        self.theta = self.theta + self.omega*dt

        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(-self.theta),1)
        self.rect = self.rotated.get_rect(center=(self.x,self.y))

        if self.leader:
            if event is not None:
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_KP4:
                        # self.vl = self.vl + 0.001*self.meter2pixel
                        self.u = self.u + 0.001*self.meter2pixel
                    elif event.key == pygame.K_KP1:
                        # self.vl = self.vl - 0.001*self.meter2pixel
                        self.u = self.u - 0.001*self.meter2pixel
                    elif event.key == pygame.K_KP6:
                        # self.vr = self.vr + 0.001*self.meter2pixel
                        self.omega = self.omega + 0.0001*self.meter2pixel
                    elif event.key == pygame.K_KP3:
                        # self.vr = self.vr - 0.001*self.meter2pixel
                        self.omega = self.omega - 0.0001*self.meter2pixel
        else:
            self.following()

        # unwind theta
        if self.theta>2*math.pi or self.theta<-2*math.pi:
            self.theta=0

        # # check for velocity saturation
        # # max
        # self.vr = min(self.vr,self.maxspeed)
        # self.vl = min(self.vl,self.maxspeed)
        # # min
        # self.vr = max(self.vr,self.minspeed)
        # self.vl = max(self.vl,self.minspeed)
#
    def draw_trail(self, pos, map, color):
        for i in range(0,len(self.trail_set)-1):
            pygame.draw.line(map,color,
                                (self.trail_set[i][0],self.trail_set[i][1]),
                                (self.trail_set[i+1][0],self.trail_set[i+1][1]))
        if self.trail_set.__sizeof__()>3000:
            self.trail_set.pop(0)
        self.trail_set.append(pos)
#
    def following(self):
        target = self.follow.trail_set[0]
        delta_x = target[0] - self.x
        delta_y = target[1] - self.y
        self.u = delta_x*math.cos(self.theta)+delta_y*math.sin(self.theta)
        self.omega = (-1/self.a)*math.sin(self.theta)*delta_x+(1/self.a)*math.cos(self.theta)*delta_y
#
    def dist(self,point1,point2):
        (x1,y1) = point1
        (x2,y2) = point2
        x1 = float(x1)
        y1 = float(y1)
        x2 = float(x2)
        y2 = float(y2)

        return ((x1-x2)**2 + (y1-y2)**2)**0.5

    def robot_simulate(self,event=None):
        self.move(event=event)
        self.draw_robot(environment.map)
        self.draw_trail((self.x,self.y),environment.map,environment.yel)

if __name__ == '__main__':

    ##############
    # initialize #
    ##############

    pygame.init()
    running = True
    iterations = 0
    dt = 0
    lasttime = pygame.time.get_ticks()

    start = (200,200)
    dims = (600,1200)
    environment = Envir(dims)

    skin = r'D:\all my files\documents\uvm\5_masters\hive\github_directory\HIVE\simple_sim\skin\tank_skin_80px_wide.png'

    num_bots = 3
    fleet = []
    # make leader
    fleet.append(Robot(start, skin, width = 80))
    fleet[0].leader = True
    # make followers
    for i in range(1,num_bots):
        robot = (Robot((start[0]-i*200, start[1]), skin, width = 80, follow=fleet[i-1]))
        fleet.append(robot)


    ###################
    # simulation loop #
    ###################

    while running:
        # if quit button is pressed, stop simulation
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
            for robot in fleet:
                if not robot.leader and iterations<1:
                    continue
                robot.robot_simulate(event)

        for robot in fleet:
            if not robot.leader and iterations<1:
                continue
            robot.robot_simulate(event)

        pygame.display.update()
        environment.map.fill(environment.black)
        dt = (pygame.time.get_ticks()-lasttime)/1000
        lasttime = pygame.time.get_ticks()
        iterations = iterations + 1

        # environment.draw_ref_frame((robot.x,robot.y),robot.theta)
        # environment.draw_trail((robot.x,robot.y))
        environment.write_uomega_info(int(fleet[0].u),
                                fleet[0].omega,
                                fleet[0].theta)
