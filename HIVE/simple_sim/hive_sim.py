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
        self.height = dimensions[0]
        self.width = dimensions[1]

        # window settings
        pygame.display.set_caption('Differential Drive Robot')
        self.map=pygame.display.set_mode((self.width,self.height))

        # text vars
        self.font = pygame.font.Font('freesansbold.ttf',25)
        self.text = self.font.render('default',True,self.white,self.black)
        self.textRect = self.text.get_rect()
        self.textRect.center=(75,25) # x,y

        self.trail_set = []

    def write_info(self, Vl, Vr, theta):
        txt = f'Vl = {Vl} Vr = {Vr} theta = {int(math.degrees(theta))}'
        self.text = self.font.render(txt,True,self.white,self.black)
        self.map.blit(self.text,self.textRect)


    def draw_trail(self, pos):
        for i in range(0,len(self.trail_set)-1):
            pygame.draw.line(self.map,self.yel,
                                (self.trail_set[i][0],self.trail_set[i][1]),
                                (self.trail_set[i+1][0],self.trail_set[i+1][1]))
        if self.trail_set.__sizeof__()>10000:
            self.trail_set.pop(0)
        self.trail_set.append(pos)

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
    def __init__(self,startpos,robotImg,width):
        self.meter2pixel=3779.52

        # robot dims
        self.w=width # width in pixels
        self.x=startpos[0]
        self.y=startpos[1]
        self.theta=0
        # initial speed at 0.01 m/s, convert to pixel/s
        self.vl=0.01*self.meter2pixel
        self.vr=0.01*self.meter2pixel
        self.maxspeed=+0.02*self.meter2pixel
        self.minspeed=-0.02*self.meter2pixel

        # skin
        self.img=pygame.image.load(robotImg)
        # every time we rotate image we deteriorate the image.
        # as such, we will hold the original image in self.img and use self.rotated
        # to always be only a single rotation command on self.img. this way we
        # do not accumulate deterioration of the image
        self.rotated=self.img
        self.rect=self.rotated.get_rect(center=(self.x,self.y))


    def draw(self,map):
        map.blit(self.rotated,self.rect)


    def move(self, event=None):
        if event is not None:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_KP4:
                    self.vl = self.vl + 0.001*self.meter2pixel
                elif event.key == pygame.K_KP1:
                    self.vl = self.vl - 0.001*self.meter2pixel
                elif event.key == pygame.K_KP6:
                    self.vr = self.vr + 0.001*self.meter2pixel
                elif event.key == pygame.K_KP3:
                    self.vr = self.vr - 0.001*self.meter2pixel

        self.x = self.x + ((self.vl+self.vr)/2)*math.cos(self.theta)*dt
        # use minus sign for y because
        self.y = self.y - ((self.vl+self.vr)/2)*math.sin(self.theta)*dt
        self.theta = self.theta + (self.vr-self.vl)/self.w*dt

        # unwind theta
        if self.theta>2*math.pi or self.theta<-2*math.pi:
            self.theta=0

        # check for velocity saturation
        # max
        self.vr = min(self.vr,self.maxspeed)
        self.vl = min(self.vl,self.maxspeed)
        # min
        self.vr = max(self.vr,self.minspeed)
        self.vl = max(self.vl,self.minspeed)


        self.rotated = pygame.transform.rotozoom(self.img,
                                                    math.degrees(self.theta),
                                                    1)
        self.rect = self.rotated.get_rect(center=(self.x,self.y))


if __name__ == '__main__':

    # initialize
    pygame.init()

    # start position
    start = (200,200)

    # dimensions
    dims = (600,1200)

    # run flag
    running = True

    # make envrionment
    environment = Envir(dims)

    # make robot
    robot=Robot(start,
                r'D:\all my files\documents\uvm\5_masters\hive\github_directory\HIVE\simple_sim\skin\tank_skin_80px_wide.png',
                # r'D:\all my files\documents\uvm\5_masters\hive\github_directory\HIVE\simple_sim\skin\ddr.png',
                80)

    dt = 0
    lasttime = 0
    # simulation loop
    while running:
        # if quit button is pressed, stop simulation
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
            robot.move(event)
        dt = (pygame.time.get_ticks()-lasttime)/1000
        lasttime = pygame.time.get_ticks()
        pygame.display.update()
        environment.map.fill(environment.black)
        robot.move()
        robot.draw(environment.map)
        environment.draw_ref_frame((robot.x,robot.y),robot.theta)
        environment.draw_trail((robot.x,robot.y))
        environment.write_info(int(robot.vl),
                                int(robot.vr),
                                robot.theta)
