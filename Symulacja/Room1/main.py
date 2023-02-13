# Loading the pygame package
import pygame

# Importing locals
from pygame.locals import *

# Other packages
import sys
import numpy as np
import numpy.random as random
import math
import time
from additional_functions import *
from dataset import positionmatrix,nr_agents

data_matrix = np.zeros((nr_agents, 4))

# Making sure we can run experiments one by one
j = 0 # add one after running

# Checking if all are executed
if j == nr_agents:
    print("nr of experiments reached")
    sys.exit()

# Initializing Pygame and font
pygame.init()
pygame.font.init() 
timefont = pygame.font.SysFont('Gothic', 30)

""" 

Creating a screen with a room that is smaller than screen 

"""

# Size of the screen
width = 800
height = 800  
size = width, height # Do not adjust this

# Creating screen
roomscreen = pygame.display.set_mode(size)

# Making background white and creating colors
WHITE = (255,255,255)
RED = (255,0,0)
GREEN = (0,255,0)
BLACK = (0,0,0)
background_color = WHITE
roomscreen.fill(background_color)
pygame.display.update()

# Defining clock
clock = pygame.time.Clock()

# Creating evacuation object class
class Agent(object):
    def __init__(self):
        
        self.mass = random.uniform(40,90)
        self.radius = 20
        # random initialize a agent
        
        self.x = random.uniform(100 + self.radius, 600 - self.radius)
        self.y = random.uniform(100 + self.radius,700 - self.radius)
        self.pos = np.array([self.x, self.y])
        #self.pos = np.array([10.0, 10.0])
    
        self.aVelocityX = random.uniform(0,1.6)
        self.aVelocityY = random.uniform(0,1.6)
        self.aVelocity = np.array([self.aVelocityX, self.aVelocityY])
        #self.actualV = np.array([0.0, 0.0])
    
        self.dest = np.array([random.uniform(100,700),random.uniform(100,700)])
        self.direction = normalize(self.dest - self.pos)
        #self.direction = np.array([0.0, 0.0])
        
        self.dSpeed = 12
        self.dVelocity = self.dSpeed*self.direction
        
        self.acclTime = 0.5 #random.uniform(8,16) #10.0
        self.drivenAcc = (self.dVelocity - self.aVelocity)/self.acclTime
              
        
        self.bodyFactor = 120000
        self.F = 2000
        self.delta = 0.08*50 #random.uniform(0.8,1.6) #0.8 #0.08
        
        self.Goal = 0
        self.time = 0.0
        self.countcollision = 0
    	
        print('X and Y Position:', self.pos)
        print('self.direction:', self.direction)
        
    def velocity_force(self): # function to adapt velocity
        deltaV = self.dVelocity - self.aVelocity
        if np.allclose(deltaV, np.zeros(2)):
            deltaV = np.zeros(2)
        return deltaV*self.mass/self.acclTime
    
    
    def f_ij(self, other): # interaction with people
        r_ij = self.radius + other.radius
        d_ij = np.linalg.norm(self.pos - other.pos)
        e_ij = (self.pos - other.pos)/d_ij
        value = self.F*np.exp((r_ij-d_ij)/(self.delta))*e_ij
        + self.bodyFactor*g(r_ij-d_ij)*e_ij
        
        if d_ij <= r_ij:
            self.countcollision += 1
            
        return value
    
    def f_ik_wall(self, wall): # interaction with the wall in the room
        r_i = self.radius
        d_iw,e_iw = distance_agent_to_wall(self.pos,wall)
        value = -self.F*np.exp((r_i-d_iw)/self.delta)*e_iw # Assume wall and people give same force
        + self.bodyFactor*g(r_i-d_iw)*e_iw
        return value
    
    def update_dest(self):
        dist = math.sqrt((self.pos[0]-700)**2 + (self.pos[1]-400)**2)
        if dist < 300:
            self.dest = np.array([700,400])
        else:
            self.dest = np.array([700,400])
            


if __name__ == "__main__":
    # Now to let multiple objects move to the door we define
    # nr_agents = nr_agents
    agent_color = RED
    line_color = BLACK
    
    # Making room
    room_height = 600 # height of the room
    room_width = 600 # width of the room
    room_left = 100 # left pixels coordinate
    room_top = 100 # top pixels coordeinate
    
    """ 
    
    Now we need to create the doors through which objects will leave in case of evacuation
    This door's position can be determined using:
    
    """
    # Door 1
    door_ytop = 382 #376
    door_ybottom = 418 #424
    
    
    # This gives the following walls
    walls = [[room_left, room_top, room_left + room_width, room_top], 
    [room_left, room_top+room_height, room_left, room_top], 
    [room_left, room_top+room_height, room_left + room_width, room_top+ room_height],
    [room_left + room_width, room_top, room_left + room_width, door_ytop],
    [room_left+room_width, room_top + room_height, room_left + room_width, door_ybottom]]
      
    # initialize agents
    agents = []
    
    def positions(agents):
        for i in range(nr_agents):
            agent = Agent()
            agent.walls = walls
            agent.x = positionmatrix[j*nr_agents+i][0]
            agent.y = positionmatrix[j*nr_agents+i][1]
            agent.pos = np.array([agent.x, agent.y])
            agent.radius = positionmatrix[j*nr_agents+i][2]
            agent.mass = positionmatrix[j*nr_agents+i][3]
            agent.dSpeed = positionmatrix[j*nr_agents+i][4]
            agents.append(agent)
        
    
    positions(agents)    
    
    count = 0
    start_time = time.time()
    run = True
    
    while run:
        
        # Updating time
        if count < nr_agents - 2:
            current_time = time.time()
            elapsed_time = current_time - start_time
        else:
            for agent_i in agents:
                data_matrix[(j+1)*nr_agents - 2][0] = elapsed_time
                data_matrix[(j+1)*nr_agents - 1][0] = elapsed_time
                agents.remove(agent_i)
            for k in range(j*nr_agents, (j+1)*nr_agents):
                data_matrix[k][1] = elapsed_time
        
        # Finding delta t for this frame
        dt = clock.tick(70)/1000
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

        
        roomscreen.fill(background_color)
        
        # Draw walls
        for wall in walls:
            start_posw = np.array([wall[0],wall[1]])
            end_posw = np.array([wall[2],wall[3]])
            start_posx = start_posw 
            end_posx = end_posw
            pygame.draw.line(roomscreen, line_color, start_posx, end_posx, 3)
        
        for agent_i in agents:
            agent_i.update_dest()
            agent_i.direction = normalize(agent_i.dest - agent_i.pos)
            agent_i.dVelocity = agent_i.dSpeed*agent_i.direction
            aVelocity_force = agent_i.velocity_force()
            people_interaction = 0.0
            wall_interaction = 0.0
        
            for agent_j in agents: 
                if agent_i == agent_j: continue
                people_interaction += agent_i.f_ij(agent_j)
        
            for wall in walls:
                wall_interaction += agent_i.f_ik_wall(wall)
            
            sumForce = aVelocity_force + people_interaction + wall_interaction
            dv_dt = sumForce/agent_i.mass
            agent_i.aVelocity = agent_i.aVelocity + dv_dt*dt 
            agent_i.pos = agent_i.pos + agent_i.aVelocity*dt
            
        # Draw agents
        for agent_i in agents:
            
            agent_i.time += clock.get_time()/1000 
            start_position = [0, 0]
            start_position[0] = int(agent_i.pos[0])
            start_position[1] = int(agent_i.pos[1])
            
            end_position = [0, 0]
            end_position[0] = int(agent_i.pos[0] + agent_i.aVelocity[0])
            end_position[1] = int(agent_i.pos[1] + agent_i.aVelocity[1])
        
            end_positionDV = [0, 0]
            end_positionDV[0] = int(agent_i.pos[0] + agent_i.dVelocity[0])
            end_positionDV[1] = int(agent_i.pos[1] + agent_i.dVelocity[1])
        
            if start_position[0] >= 699 and agent_i.Goal == 0:
                agent_i.Goal = 1
                data_matrix[count+j*nr_agents][0] =  agent_i.time
                # print('Time to Reach the Goal:', agent_i.time)
            
            if start_position[0] > 699 or start_position[0] < 100:
                data_matrix[count+j*nr_agents][2] = count 
                data_matrix[count+j*nr_agents][3] = agent_i.countcollision 
                count += 1
                agents.remove(agent_i)
            
            pygame.draw.circle(roomscreen, agent_color, start_position, round(agent_i.radius), 3)
           
        
        # Present text on screen
        timestr = "Time: " +  str(elapsed_time)
        timesurface = timefont.render(timestr, False, (0, 0, 0))
        roomscreen.blit(timesurface,(0,0))
        # Update the screen
        pygame.display.flip()
        
    pygame.quit()


