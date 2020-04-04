# -*- coding: utf-8 -*-
"""
Created on Sun Mar  1 01:33:12 2020

@author: akpo2
"""

import numpy as np
import matplotlib.pyplot as plt

# Defining the maze wherein 1's represent the free path and 0's represent the obstacles
def GetMaze():
    return np.array([[ 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 ], 
		[ 1, 0, 1, 0, 1, 1, 1, 0, 1, 1 ], 
		[ 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 ], 
		[ 0, 0, 1, 0, 1, 0, 1, 0, 0, 1 ], 
		[ 1, 1, 1, 0, 1, 1, 1, 1, 1, 0 ], 
		[ 1, 1, 1, 1, 1, 1, 0, 1, 1, 0 ], 
		[ 1, 1, 1, 1, 1, 0, 0, 0, 1, 1 ], 
		[ 1, 1, 0, 1, 1, 1, 0, 1, 1, 1 ], 
		[ 1, 1, 1, 0, 1, 1, 1, 1, 0, 1 ]])
#    return np.array([[1,1,1,1],
#                     [1,1,0,0],
#                     [1,1,1,1]])

# Function to get the neighbours of the given node
# here we get the 4-connected neighbours
def getNeighbours(point):
    x_row=np.array([[-1,0],[0,-1],[0,1],[1,0]])
    #y_row=np.array([[0],[-1],[1],[0]])
    neigh=np.reshape(np.zeros(x_row.shape[0]*2),(4,2))
    neigh=point+x_row
    return neigh

# Here we check whether a given point is valid or not
# there are multiple conditions such as x,y co-ordinatedes of the node are not greater than
# the maze x,y limits
# the other condition is that the both x,y co-ordinates of the node to be checked are greater than 0
# next condition is that we check whether the node is in obstacle region or in free space
# the obstacle region is denoted by 0 and free space by 1
def validPoint(maze,node):
    m=int(node[0][0])
    h_y=int(node[0][1])
    if(m>=0 and m<maze.shape[0] and h_y>=0 and h_y<maze.shape[1] and maze[m][h_y]==1 and maze[m][h_y]!=0):
        return True
    else:
        return False

# Here we check whether the given node is in visited list or not
def checkVisited(node,visited_list):
    for samples in visited_list:
        if(np.allclose(samples,node)):
            return True
    return False

#this function is used for printing the maze, path found
def printPath(path_found,maze,source,destination,explored):
    x_path=[]
    y_path=[]
    x_explored=[]
    y_explored=[]
    for i in path_found:
        i=np.reshape(i,(2,1))
        x_path.append(i[0][0])
        y_path.append(i[1][0])
    for i in explored:
        i=np.reshape(i,(2,1))
        x_explored.append(i[1][0])
        y_explored.append(i[0][0])
        #modified_maze[k_x][k_y]=8
    fig, ax = plt.subplots()
    ax.set_yticks([1, 1, 1], minor=False)
    plt.imshow(maze, cmap=plt.get_cmap('gray'))
    #plt.grid('True',linewidth=1)
    plt.plot(y_path, x_path)
    plt.plot(x_explored, y_explored, 'ro')
    types=['source','destination']
    x_goal = [source[0][1],destination[0][1]] 
    # y-axis values 
    y_goal = [source[0][0],destination[0][0]] 
    
    for i,type in enumerate(types):
        x = x_goal[i]
        y = y_goal[i]
         # plotting points as a scatter plot 
        plt.scatter(x_goal, y_goal, label= "stars", color= "green",  
                    marker= "*", s=90) 
        plt.text(x+0.2, y-0.7, type, fontsize=10)
        plt.title("DFS")


# Here the main algorithm is implemented
def DFS(source,destination,maze):
    #maze=GetMaze()
    visited=[]
    explored=[]
    to_be_visited=[]
    to_be_visited.append(source)
    queue_path=[]
    visited.append(source)
    path=list(source)
    # Get the neighburs of the source
    neighbour_nodes=getNeighbours(source)
    # Add the nieghbours of source to the open list
    to_be_visited.extend(neighbour_nodes)
    
    while(len(to_be_visited)!=0):
        # Take out the last element from the open list
        node=to_be_visited.pop(-1)
        
        node=np.reshape(node,(1,2))
        #explored.append(node)
        # Check whether node is valid or not
        if(validPoint(maze,node) and (not(checkVisited(node,visited)))):
            # Get the neighbours of the node
            neighbour_nodes=getNeighbours(node)
            # add the neighbour to the open list
            to_be_visited.extend(neighbour_nodes)
            path.append(node)
            explored.append(node)
            queue_path.append(path)
            # if the neighbour is destination, return the path
            if((node==destination).all()):
                return path,explored
        # Add the explored node to the closed list
        visited.append(node)
    return 0,explored

# Define the source and destination
source=np.array([[0,0]])
destination=np.array([[8,9]])
maze=GetMaze()
path_found,explored=DFS(source,destination,maze)
if(path_found==0):
    print("No path exists between source and destination")
else:
    print("The path found is")
    print(printPath(path_found,maze,source,destination,explored))
    print("The cost of the path is : ")
    print(len(path_found))