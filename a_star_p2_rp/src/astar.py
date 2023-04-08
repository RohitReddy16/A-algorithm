#!/usr/bin/env python3
#Importing the library
import numpy as np
import pygame
from queue import PriorityQueue
import re
import time
import math
from geometry_msgs.msg import Twist
import rospy

# Initializing the three used colors
color = (255,255,255)
color_2 = (255,200,150)
color_3=(0,0,0)

# Initializing the map
pygame.init()
width_, height_ = 600, 200

# Initializing surface
surface = pygame.Surface((width_,height_))
surface.fill(color_2)
goal=None # goal

# Function for action set
# Based on the given actionset the new node and the corresponding values are calculated
def move(lst,RPM):
    def cost_c(Xi,Yi,Thetai,UL,UR):
        way=[] # path to avoide going through obstackles and to curve the path
        way_ros=[]# ros visualization
        t = 0
        r = 0.033*100
        L = 0.16*100
        dt = 0.1
        Xn=Xi
        Yn=Yi
        Thetan = 3.14 * Thetai / 180
    # Xi, Yi,Thetai: Input point's coordinates
    # Xn, Yn, Thetan: End point coordintes
        D=0
        way.append((round(Xn),round(Yn)))
        way_ros.append([round(Xn),round(Yn),round(Thetai),RPM])
        while t<1:
            t = t + dt
            Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
            Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
            Thetan += (r / L) * (UR - UL) * dt
            D=D+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
            way.append((round(Xn),round(Yn)))
            way_ros.append([round(Xn),round(Yn),round(180 * (Thetan) / 3.14),RPM])
        Thetan = 180 * (Thetan) / 3.14
        return round(Xn), round(Yn), round(Thetan), D, way,way_ros
        
    # Extracting the information
    coords=list(lst[3])
    cost2come=lst[5]
    x_s=coords[0]
    y_s=coords[1]
    theta_t=lst[4]
    # Generating the result
    x_n,y_n,theta_n,c_n,rode,way_ros=cost_c(x_s,y_s,theta_t,RPM[0],RPM[1])
    
    cost2go=math.dist((x_n,y_n),goal)
    new_cost_2_come=cost2come+c_n
    cost=cost2go+new_cost_2_come
    return(tuple((x_n,y_n)), theta_n,cost,rode,new_cost_2_come,cost2go,way_ros)

# Start the algorithm, ask for user input in the given format, out of reachable points
while True:
    print("Enter clearance (e.g. 5): ")
    user_input = input()
    match = re.match(r'^\s*(\d+)\s*$', user_input)
    if match:
        clearance = int(match.group(1))
        radius = 10.5 ### change this
        if clearance < 0:
            print("Clearance  must be positive. Please try again.")
        else:
            cr=clearance+radius
            cr2=radius-clearance
            #Drawing the map with obstacles
            pygame.draw.rect(surface, color, pygame.Rect(cr, cr, width_-2*cr, height_-2*cr))

            # Define the hexagon in the center with original dimensions
            pygame.draw.rect(surface, color, pygame.Rect(cr, cr, width_-2*cr, height_-2*cr))
            bottom_rect_dim = [(250-cr2,200),(265+cr,200),(265+cr,75-cr2),(250-cr2,75-cr2)]
            pygame.draw.polygon(surface, color_2,bottom_rect_dim)
            top_rect_dim = [(150-cr2,0),(165+cr,0),
                            (165+cr,125+cr),(150-cr2,125+cr)]
            pygame.draw.polygon(surface,color_2,top_rect_dim)

            # Drawing a circle
            pygame.draw.circle(surface, color_2,(400,90),50+cr)


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
        print("Invalid input. Please enter clearance, which should be positive integer.")

# Loop to get start nodes and check if it is a valid start node.
while True:
    print("Enter start x,y,theta coordinates (e.g. 2,3,60): ")
    user_input = input()
    match = re.match(r'^\s*(\d+)\s*,\s*(\d+)\s*,\s*(-?[0-9]*[0-9]*0)\s*$', user_input)
    if match:
        x = int(match.group(1))
        y = 200-int(match.group(2))
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

# Loop to get gosl nodes and check if it is a valid goal node
while True:
    print("Enter goal x,y coordinates (e.g. 2,3): ")
    user_input = input()
    match = re.match(r'^\s*(\d+)\s*,\s*(\d+)\s*', user_input)
    if match:
        x = int(match.group(1))
        y = 200-int(match.group(2))
        if arr[x, y] == 1:
            print("Goal is inside of an obstacle, please try again")
        else:
            goal = (x, y) #goal position
            break
    else:
        print("Invalid input. Please enter x,y coordinates in the format 'x,y'.")

# Loop to get the RPM values.
while True:
    print("Enter the RPM values separated by comma(e.g. 100,50): ")
    user_input = input()
    match = re.match(r'^\s*(\d+)\s*,\s*(\d+)\s*', user_input)
    if match:
        RPM1= int(match.group(1))
        RPM2= int(match.group(2))
        break
    else:
        print("Invalid input. Please enter the RPM values in the format 'RPM1,RPM2'")

# Defining the require variables for the algorithm, the pixels is a dictionary for the explored nodes
pixels={} # closed list
exploration={} #list to get exploration nodes of path
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
ros={}
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
        print("Final cost:",first[0])
        end_time=time.time()# algorithm end time
        break
    # Looping the eight different actions 
    dir=[[0,RPM1],[RPM1,0],[RPM1,RPM1],[0,RPM2],[RPM2,0],[RPM2,RPM2],[RPM1,RPM2],[RPM2,RPM1]] # possible theta values
    for i in range(0,8):
        coords,angle,cost,rode,c1,c2,ros_temp=move(first,dir[i])
        
        # Checking if the new pixel is in the obstacle space or it was already explored, or the path crosses an obstacle or it is not on the map
        if ((not(any(arr[x, y] == 1 for x, y in rode))) and 
            ((coords[0]>0 and coords[0]<600)and(coords[1]>0 and coords[1]<200)) and 
            (not(coords in obstacles)) and (not(coords in closed))):
            # Adding it to the queue if it was not there yet
            if not(coords in global_dict):
                ros[coords]=ros_temp
                # Updating the all noode list, the queue and the line vector while increasing the child node index
                global_dict[coords]=[cost, child, parent, coords,angle,c1,c2]
                Q.put(global_dict[coords])
                line_vector[first[3]].append(coords)
                child += 1 
                exploration[coords]=rode

            # Updating the dictionary if the coordinate is found with lower cost
            else:
                if(global_dict[coords][5]>c1):
                    global_dict[coords][5]=c1
                    global_dict[coords][2]=parent
                    global_dict[coords][0]=cost
                    global_dict[coords][4]=angle
                    exploration[coords]=rode
                    ros[coords]=[ros_temp]
        
# Creating the end display 
s=pygame.display.set_mode((width_,height_))
s.blit(surface,(0,0))
pygame.display.update()

# Showing the exploration for the map
for k in exploration.keys():
    pygame.draw.lines(s,(255,0,0),False,exploration[k])
    pygame.display.update()
#printing the length of pixles explored
print("Len:",len(pixels))
# Showing the optimal path if the goal was found using the parent child relationship stored in the dictionary
if(not (Q.empty())):   
    value = g_key
    # Backtrack and generate the solution path
    path=[]
    path2=[]
    ros_path=[]
    while(pixels[value][1]!=-1):
        path.append(pixels[value][2])
        
        # Curved and ROS Path
        if(pixels[value][2]!=(start)):
            path2.extend(reversed(exploration[pixels[value][2]]))
            ros_path.extend(reversed(ros[pixels[value][2]]))
            # print("k")
        value=pixels[value][1]
    path.append(pixels[value][2])
    path.reverse()
    path2.reverse()
    ros_path.reverse()
    print(ros_path)
    
    # # Displaying the path with blue 
     
    for i,walk in enumerate(path2):
        if(i+1>len(path2)-1):
            break        
        pygame.draw.line(s,(0,0,255),walk,path2[i+1],width=1)
        pygame.display.update()
       
# Printing the time used by the algorithm
print('Time:',end_time-start_time)
# Showing the screen
running = True
pygame.time.wait(10000)


print("I'm done till here")


rospy.init_node('ROS_AStar', anonymous=True)
pub_vel = rospy.Publisher('cmd_vel',Twist,queue_size=100)
msg = Twist()
turtle_model = rospy.get_param("model","burger")
msg.linear.x = 0.0
msg.linear.y = 0.0
msg.linear.z = 0.0
msg.angular.x = 0.0
msg.angular.y = 0.0
msg.angular.z = 0.0

pub_vel.publish(msg)
# publishing rate
ratee = rospy.Rate(1.1)
t =1 #Time step
r = 0.033
L = 0.16

# Moving the turtlebot
for i in range(len(ros_path)):
    
    # Calculation of the linear and angular velocity
    x,y,th,rp=ros_path[i]
    dt = 1
    ul,ur = rp
    ul = ul * 2 * math.pi / 60
    ur = ur * 2 * math.pi /60
    tn = 3.14 * th/180
    t_dot = (r/L)*(ur-ul)
    tdif = t_dot+tn
    xdot = (r/2)*(ur+ul)*math.cos(tdif)
    ydot = (r/2)*(ur+ul)*math.sin(tdif)
    vel_inp = math.sqrt(xdot**2 + ydot**2)
    msg.linear.x = vel_inp
    msg.angular.z = -t_dot
    pub_vel.publish(msg)
    ratee.sleep()

msg.linear.x = 0.0
msg.linear.y = 0.0
msg.linear.z = 0.0
msg.angular.x = 0.0
msg.angular.y = 0.0
msg.angular.z = 0.0

pub_vel.publish(msg)

# Game loop
while running:
# For loop through the event queue  
    for event in pygame.event.get():
        # Check for QUIT event      
        if event.type == pygame.QUIT:
            pygame.quit()
            running = False