# -*- coding: utf-8 -*-
"""
Created on Sat Mar  7 16:16:12 2020

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
#                     [1,1,0,1],
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
    #plt.axis([0, 6, 0, 20])
    #plt.show()
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
        plt.title("Dijkstra")


# this function calculates the distance of the node from the parent  
def calDistParent(node_child,node_parent):
    return (abs(node_parent[0][0]-node_child[0][0])+abs(node_parent[0][1]-node_child[0][1]))

# This function calculate the total cost of the node
# The total cost is the cost of the node from parent plus cost of parent from source
def calCost(parent,neighbour,initial_cost):
        cost_node=(abs(parent[0][0]-neighbour[0][0])+abs(parent[0][1]-neighbour[0][1]))
        cost_node=initial_cost+cost_node
        return cost_node
    
# This function searches for the cost of the parent in a list
def findCostPrevNode(list_cost_prevNode,searchNode):
        for cost, node in list_cost_prevNode.items():
            if ((node==searchNode).all()):
                return cost
        return 0;


# Update the cost of the node based on the new node selected in the dictionary
# which saves the cost of  nodes from the cureent selected node
def appendNodesCost(cost_cal,neighbour,to_be_visited,i,node_parent,list_parent):
        for cost, node in list(to_be_visited.items()):
            if (np.allclose(node[0],neighbour)):
                if(cost<cost_cal):
                    cost_cal=cost
                    
        temp={i:[neighbour,cost_cal]}
        to_be_visited.update(temp)
        
        for cost, node in list(list_parent.items()):
            if (np.allclose(node[0],neighbour)):
                if(cost<cost_cal):
                    cost_cal=cost
                    node_parent=node[2]
                    
        temp={i:[neighbour,cost_cal]}
        to_be_visited.update(temp)
        temp2={i:[neighbour,cost_cal,node_parent]}
        list_parent.update(temp2)
                
        return to_be_visited,list_parent;

# This function traces the path from the destination to the source   
def traceParent(listParent,source,destination):
    list_path=[]
    count=0
    node_child=destination
    list_path.append(destination)
    for i in range(len(list_parent)):
        for cost, node in list(list_parent.items()):
                if (np.allclose(node[0],node_child)):
                    
                    if(not np.allclose(source,node_child)):
                        node_child=node[2]
                        list_path.append(node_child)
                        break
                    else:
                        list_path.append(source)
                        count=1
                        break
        if(count==1):
            break;
    return list_path
    

# Implementation of the Dijkstra Algorithm
def Dijkstra(source,destination,maze):
    i=1
    path=[]
    visited=[]
    to_be_visited=[]
    list_cost_prevNodes={}
    # List keeping track of costs of nodes
    list_cost_prevNodes={0:source}
    list_neigh=[]
    list_parent={}
    # List which keeps a track of parent and child
    list_parent={i:[source,0,source]}
    #source_s=str(source)
    to_be_visited={i:[source,0]}
    while(len(to_be_visited)):
        # Dictionary of open nodes whcih has the node and the cost
        #Sort the list in descending order
        to_be_visited=dict(sorted(to_be_visited.items(), key=lambda i: i[1][1], reverse=True))
        # Take out the node with minimum distance
        last_item=to_be_visited.popitem()
        node=np.array(last_item[1])[0]
        cost=findCostPrevNode(list_cost_prevNodes,node)
        path.append(node)
        # Calculate the neighbours
        neighbour_nodes=getNeighbours(node)
        for neighbour in neighbour_nodes:
            neighbour=np.reshape(neighbour,(1,2))
            # Check the validity of the node
            if(validPoint(maze,neighbour) and (not(checkVisited(neighbour,visited)))):
                list_neigh.append(neighbour)
                # Calculate the cost of node from source
                total_cost=calCost(node,neighbour,cost)
                i=i+1
                #Update the cost of node if it is minimum from current parent as well as 
                # update the parent in the list if minimum cost found
                to_be_visited,list_parent=appendNodesCost(total_cost,neighbour,to_be_visited,i,node,list_parent)
        # Add the node to closed list
        visited.append(node)
        #print(visited)
        
    return list_parent,list_neigh
    
            
            
# Define the source destination
source=np.array([[0,0]])
destination=np.array([[8,9]])
maze=GetMaze()
list_parent,explored=Dijkstra(source,destination,maze)
path_found=traceParent(list_parent,source,destination)
if(path_found==0):
    print("Path not found")
else:
    print(printPath(path_found,maze,source,destination,explored))
