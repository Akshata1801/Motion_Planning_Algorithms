# -*- coding: utf-8 -*-
"""
Created on Mon Feb 24 13:10:52 2020

@author: akpo2
"""
import numpy as np
import matplotlib.pyplot as plt


# Defining the maze wherein 1's represent the free path and 0's represent the obstacles
#def GetMaze():
#    return np.array([[ 1, 1, 1, 1, 1 ], 
#		[ 1, 1, 1, 1, 1], 
#		[ 1, 1, 1, 0, 1 ], 
#		[ 1, 1, 1, 1, 1], 
#		[ 1, 1, 1, 1, 1]])
#    return np.array([[1,1,1,1],
#                     [1,1,0,0],
#                     [1,1,1,1]])
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
#  
    
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
    if(m>=0 and m<maze.shape[0] and h_y>=0 and h_y<maze.shape[1] and maze[m][h_y]==1):
        return True
    else:
        return False

# Here we check whether the given node is in visited list or not
def checkVisited(node,visited_list):
    for samples in visited_list:
        if(np.allclose(samples,node)):
            return True
    return False

# this function is used for printing the maze, path found
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
        x_explored.append(i[0][0])
        y_explored.append(i[1][0])
        #modified_maze[k_x][k_y]=8
    fig, ax = plt.subplots()
    ax.set_yticks([1, 1, 1], minor=False)
    
    plt.imshow(maze, cmap=plt.get_cmap('gray'))
    #plt.grid('True',linewidth=1)
    plt.plot(y_path, x_path)
    plt.plot(y_explored, x_explored, 'ro')
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
        plt.title("BFS")
        #return modified_maze
    
        
# This function implements the actual BFS Algorithm
def BFS(source,destination,maze):
    maze=GetMaze()
    visited=[]
    explored=[]
    to_be_visited=[]
    to_be_visited.append(source)
    visited.append(source)
    while(len(to_be_visited)!=0):
        # take out the queue from front of the list of queues
        path=to_be_visited.pop(0)
        # Take out the last element from the queue
        node=path[-1]
        # get the neighbours of the node
        neighbours_node=getNeighbours(node)
        #Iterate through each neighbour of the node
        for neighbour in neighbours_node:
            neighbour=np.reshape(neighbour,(1,2))
            # if the neighbour satisfies all the conditions of validity
            # then insert the neighbour in the queue
            if(validPoint(maze,neighbour) and (not(checkVisited(neighbour,visited)))):
               
                new_path=list(path)
                new_path.append(neighbour)
                to_be_visited.append(new_path)
                explored.append(neighbour)
                # If the neighbouring node is destination
                # then return the path
                if((neighbour==destination).all()):
                    return new_path,explored
        # Add the node to the visited list
        visited.append(node)
        #print(new_path)
    return 0,explored


# Define the source and destination
source=np.array([[0,0]])
destination=np.array([[8,9]])
maze=GetMaze()
path_found,explored=BFS(source,destination,maze)
if(path_found==0):
    print("No path exists between source and destination")
else:
    print("The path found is")
    printPath(path_found,maze,source,destination,explored)
    print("The cost of the path is : ")
    print(len(path_found))


    