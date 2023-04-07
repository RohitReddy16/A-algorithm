#Importing the library
import numpy as np
import pygame
from queue import PriorityQueue
import re
import time
import math

# Initializing the three used colors
color = (255,255,255)
color_2 = (255,200,150)
color_3=(0,0,0)

# Initializing the map
pygame.init()
width_, height_ = 600, 250

# Initializing surface
surface = pygame.Surface((width_,height_))
surface.fill(color_2)
goal=None # goal
l=None #step size

# Checking the path to do not go through obstackles, all the points are calculated between the start and end point
def bresenham_line(x0, y0, x1, y1):
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while x0 != x1 or y0 != y1:
        points.append((x0, y0))
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

    points.append((x0, y0))
    return points


# Function for action set
# 0:2left,1:left,2:forward,3:right,4:2right:
def move(lst,thet):
    # Extracting the information
    coords=list(lst[3])
    cost2come=lst[5]+l
    x_s=coords[0]
    y_s=coords[1]
    theta_t=lst[4]
    # Generating the result
    theta_sum=thet+theta_t
    x_n=round(x_s+(l*np.cos(np.deg2rad(theta_sum))))
    y_n=round(y_s+(l*np.sin(np.deg2rad(theta_sum))))
    rode = bresenham_line(x_s,y_s,x_n,y_n)
    cost2go=math.dist((x_n,y_n),goal)
    cost=cost2go+cost2come
    return(tuple((x_n,y_n)), theta_sum,cost,rode,cost2come,cost2go)

# Start the algorithm, ask for user input in the given format, out of reachable points
while True:
    print("Enter clearance and radius values separated by a space (e.g. 5 10): ")
    user_input = input()
    match = re.match(r'^\s*(\d+)\s+(\d+)\s*$', user_input)
    if match:
        clearance = int(match.group(1))
        radius = int(match.group(2))
        if clearance < 0 or radius < 0:
            print("Clearance and radius values must be positive. Please try again.")
        else:
            cr=clearance+radius
            # Define the hexagon in the center with original dimensions
            pygame.draw.rect(surface, color, pygame.Rect(cr, cr, width_-2*cr, height_-2*cr))
            bottom_rect_dim = [(150+cr,150-cr),(150+cr,250),(100-cr,250),(100-cr,150-cr)]
            pygame.draw.polygon(surface, color_2,bottom_rect_dim)
            top_rect_dim = [(100-cr,0),(150+cr,0),(150+cr,100+cr),(100-cr,100+cr)]
            pygame.draw.polygon(surface,color_2,top_rect_dim)

            # Vertices of hexagon
            hexagon_dim = [(300,50-cr),(364.95190528+cr,87.5-((cr)*np.tan(np.pi*30/180))),
                        (364.95190528+cr,162.5+((cr)*np.tan(np.pi*30/180))),(300,200+cr),
                        (235.04809472-cr,162.5+((cr)*np.tan(np.pi*30/180))),
                        (235.04809472-cr,87.5-((cr)*np.tan(np.pi*30/180)))]
            pygame.draw.polygon(surface,color_2,hexagon_dim)

            # Define the triangle with the original dimensions
            triangle_dim = [(460-cr,25-((cr)/np.tan(np.pi*13.28/180))),(460.00-cr,225+((cr)/np.tan(np.pi*13.28/180))),(510+((cr)/np.cos(np.pi*26.5650518/180)),125)]
            pygame.draw.polygon(surface,color_2,triangle_dim)

            # Convert surface to a 2D array with 0 for the specific color and 1 for other colors
            arr = np.zeros((surface.get_width(), surface.get_height()))
            pix = pygame.surfarray.pixels3d(surface)
            arr[np.where((pix == color_2).all(axis=2))] = 1
            obs_np = np.where((pix == color_2).all(axis=2))
            obstacles={}
            for i in range(obs_np[0].shape[0]):
                obs_key = (obs_np[0][i], obs_np[1][i])  # Create a new tuple (y, x)
                obstacles[obs_key] = None  # Use the tuple as a key and assign a value of None
            del pix
            break
    else:
        print("Invalid input. Please enter clearance and radius values separated by a space, both of which should be positive integers.")

# Loop for giving step size input and checking the input is valid
while True:
    print("Enter a step size between 1 and 10: ")
    user_input = input()
    match = re.match(r'^\s*(\d+)\s*$', user_input)
    if match:
        l = int(match.group(1))
        if l < 1 or l > 10:
            print("Step size should be a positive integer between 1 and 10. Please try again.")
        else:
            break
    else:
        print("Invalid input. Please enter a positive integer between 1 and 10 as the step size.")

# Loop to get start nodes and check if it is a valid start node.
while True:
    print("Enter start x,y,theta coordinates (e.g. 2,3,60): ")
    user_input = input()
    match = re.match(r'^\s*(\d+)\s*,\s*(\d+)\s*,\s*(-?[0-9]*[0-9]*0)\s*$', user_input)
    if match:
        x = int(match.group(1))
        y = 250-int(match.group(2))
        theta = int(match.group(3))
        if not (-180 <= theta <= 180 and theta % 30 == 0) and (x>0 and x<600) and (y>0 and y<250):
            print(" Invalid input for theta. Please enter an angle in degrees between -180 and 180 that is a multiple of 30 degrees.")
            continue
        if arr[x, y] == 1:
            print("Start is inside of an obstacle, please try again")
        else:
            start = (x, y) # start position
            s_theta=theta# start theta
            break
    else:
        print("Invalid input. Please enter x,y,theta coordinates in the format 'x,y,theta', where theta is an angle in degrees and a multiple of 30 degrees from -180 to 180.")

#Loop to get start nodes and check if it is a valid goal node
while True:
    print("Enter goal x,y,theta coordinates (e.g. 2,3,60): ")
    user_input = input()
    match = re.match(r'^\s*(\d+)\s*,\s*(\d+)\s*,\s*(-?[0-9]*[0-9]*0)\s*$', user_input)
    if match:
        x = int(match.group(1))
        y = 250-int(match.group(2))
        theta = int(match.group(3))
        if not (-180 <= theta <= 180 and theta % 30 == 0)and (x>0 and x<600) and (y>0 and y<250):
            print("Invalid input for theta. Please enter an angle in degrees between -180 and 180 that is a multiple of 30 degrees.")
            continue
        if arr[x, y] == 1:
            print("Goal is inside of an obstacle, please try again")
        else:
            goal = (x, y) #goal position
            g_theta=theta #goal theta
            break
    else:
        print("Invalid input. Please enter x,y,theta coordinates in the format 'x,y,theta', where theta is an angle in degrees and a multiple of 30 degrees from -180 to 180.")


# Defining the require variables for the algorithm, the pixels is a dictionary for the explored nodes
pixels={} # closed list
c2c=0 # cost to come
c2g=math.dist(start,goal) # cost to go
d1 = [c2g+c2c, 0, -1,start, s_theta,c2c,c2g] # list stores all the information
Q = PriorityQueue()# Queue
global_dict={} # dictionary to store information about nodes in graph
global_dict[start]=d1
Q.put(d1)
parent=-1 # parent node index
child=1# child node index
closed={}

# Start the timer
start_time=time.time()
# Dictionary for visualization of the exploration
line_vector = {}
# The A * algorithm to find the shortest path 
while(True):

    #Check if there is any pixel that we haven't visited yet  
    if(Q.empty()):
        print("Goal is unreachable")
        end_time=time.time() # start timer
        break
    # Popping the pixel with the lowest cost and adding it to the dictionary
    first = Q.get() # element with the lowest cost
    line_vector[first[3]]=[]
    pixels[first[1]]=[first[0],first[2],first[3],first[4],first[5],first[6]]
    parent=first[1]
    closed[first[3]]=None  
    # Checking if the goal is reached
    if(math.dist(first[3],goal)<=0.5):
        g_key=first[1] # index of the goal node
        print("Goal reached")
        end_time=time.time()# algorithm end time
        break
    # Looping the five different actions 
    szog=[-60, -30,0, 30, 60] # possible theta values
    for i in range(0,5):
        thet=szog[i]
        coords,angle,cost,rode,c1,c2=move(first,thet)
        # Checking if the new pixel is in the obstacle space or it was already explored, or the path crosses an obstacle or it is not on the map
        if ((not(any(arr[x, y] == 1 for x, y in rode))) and 
            ((coords[0]>0 and coords[0]<600)and(coords[1]>0 and coords[1]<250)) and 
            (not(coords in obstacles)) and (not(coords in closed))):
            # Adding it to the queue if it was not there yet
            if not(coords in global_dict):
                
                # Updating the all noode list, the queue and the line vector while increasing the child node index
                global_dict[coords]=[cost, child, parent, coords,angle,c1,c2]
                Q.put(global_dict[coords])
                line_vector[first[3]].append(coords)
                child += 1 
                

            # Updating the dictionary if the coordinate is found with lower cost
            else:
                if(global_dict[coords][5]>c1):
                    global_dict[coords][5]=c1
                    global_dict[coords][2]=parent
                    global_dict[coords][0]=cost
                    global_dict[coords][4]=angle
    

# Creating the end display 
s=pygame.display.set_mode((width_,height_))
s.blit(surface,(0,0))
pygame.display.update()

# Showing the exploration for the map
for k in line_vector.keys():
    for i in range(0,len(line_vector[k])):
        pygame.draw.aaline(s,(255,0,0),k,line_vector[k][i])
        pygame.display.update()

print("Len:",len(pixels))
# Showing the optimal path if the goal was found using the parent child relationship stored in the dictionary
if(not (Q.empty())):   
    value = g_key
    # Backtrack and generate the solution path
    path=[]
    while(pixels[value][1]!=-1):
        path.append(pixels[value][2])
        value=pixels[value][1]
    path.append(pixels[value][2])
    path.reverse()
    
    # Displaying the path with blue 
    for i,walk in enumerate(path):
        if(i+1>len(path)-1):
            break
        pygame.draw.line(s,(0,0,255),walk,path[i+1],width=1)
        pygame.display.update()

    
# Printing the time used by the algorithm
print('Time:',end_time-start_time)
# Showing the screen
running = True
pygame.time.wait(10000)
# Game loop
while running:
# For loop through the event queue  
    for event in pygame.event.get():
        # Check for QUIT event      
        if event.type == pygame.QUIT:
            pygame.quit()
            running = False