#!/usr/bin/env python3

import numpy as np
from numpy import array, pi, abs
import rospy
import networkx as nx
from sensor_msgs.msg import JointState
from a7_prm.msg import PRMPath
from geometry_msgs.msg import PoseStamped

#Set G = nx.Graph() for later use
G = nx.Graph()

class Node:
    def __init__(self, value):
        self.value = value
        
def learningPhrase(N):
    '''
    Parameters:
        N (int): number of iterations
        
    Returns:
        networkx.graph: a graph object
    '''
    #Create N random samples, wrap them in a node object, and add them to the graph    
    for i in range(N):
        jointAngles = generateSample()
        node = Node(jointAngles)
        G.add_node(node)
        
    #Check if an edge exists between each pair of nodes for all nodes in the graph, use checkForEdge()
    #If return >= 0 edge exists and add edge to graph, else edge does not exist
    for node1 in G.nodes:
        for node2 in G.nodes:
            if node1 != node2:
                S = checkForEdge(node1, node2)
                if S >= 0:
                    G.add_edge(node1, node2, weight=S)
                    
    print('Number of edges: ', G.number_of_edges())
                
    return G

def localPlanner(config1, config2):
    '''
    For now returns True, in future will check if any obsticals exist
    '''
    return True

def find_distance_between_angles(a, b):
    ''' Returns the distance between two angles in radians (b-a)
    '''
    # -- bring values between -2*pi and +2*pi
    if (b > 2*pi or b < -2*pi):
        b = b % (2*pi)
    if (a > 2*pi or a < -2*pi):
        a = a % (2*pi)
    # -- bring values between 0 and 2*pi
    if (b < 0):
        b = b + 2*pi
    if (a < 0):
        a = a + 2*pi
    # -- find the distance
    dist = b - a
    if (dist > pi):
        dist = dist - 2*pi
    elif (dist < -pi):
        dist = dist + 2*pi
    return dist


  
def checkForEdge(node1, node2):
    '''
    Checks if an edge exists between two nodes
    Parameters:
        node1 (Node): a node object
        node2 (Node): a node object

    Returns:
        Return S if edge exists, else return -1
    '''
    #Set threshold to 0.785
    threshold = 0.785
    
    #Condition One, take abs value of the distance between angles
    fd = find_distance_between_angles
    getDst1 = abs(fd(node1.value[0], node2.value[0]))
    getDst2 = abs(fd(node1.value[1], node2.value[1]))
    getDst3 = abs(fd(node1.value[2], node2.value[2]))
    
    #print('Distances: ', distances)
    
    S = getDst1 + getDst2 + getDst3
    
    #Condition Two, currently returns True Always
    lp = localPlanner(node1.value, node2.value)
    
    if getDst1 < threshold and getDst2 < threshold and getDst3 < threshold and lp:
        #print('Edge exists between: ', node1.value, ' and ', node2.value)
        return S
    else:
        return -1 
        

def generateSample():
    '''
    Generates a random sample of joint angles
    Parameters:
        None

    Returns:
        list: a list of three joint angles
    '''
    #Define the range of joint angles
    theta1 = np.random.uniform(-np.pi, np.pi)
    theta2 = np.random.uniform(-np.pi, np.pi)
    theta3 = np.random.uniform(-np.pi, np.pi)
    
    return np.array([theta1, theta2, theta3])

def queryPhase(start, goal, g):
    '''
    Connect the start and goal nodes from the given graph
    Use a threshold of 0.785 radians
    Make a list of existing nodes and loop through
    then check if an edge exists between the start/goal and the node, and add them to the graph
    Wrap the start and goal nodes in a Node object and add them to the graph
    After connecting start and goal to the graph, use G.shortest_path to find the shortest path 
    Return the shortest path and print it
    '''
    #Define the threshold
    theshold = 0.785
    
    #Wrap the start and goal nodes in a Node object and add them to the graph
    startNode = Node(start)
    goalNode = Node(goal)
    g.add_node(startNode)
    g.add_node(goalNode)
    
    #Make a list of existing nodes
    existingNodes = list(g.nodes())
    
    #Loop through the list of existing nodes
    for node in existingNodes:
        #Check if an edge exists between the start/goal and the node
        if checkForEdge(startNode, node) >= 0:
            g.add_edge(startNode, node, weight=checkForEdge(startNode, node))
        if checkForEdge(goalNode, node) >= 0:
            g.add_edge(goalNode, node, weight=checkForEdge(goalNode, node))
        
    #Find the shortest path
    shortestPath = nx.shortest_path(G=g, source=startNode, target=goalNode)
    
    #Print the shortest path
    print('Shortest Path: ')
    for node in shortestPath:
        print(node.value)
    return shortestPath
    

def main():
    #Me
    print('In main: <Reiley Meeks> <801149501> <rmeeks3@uncc.edu>')
    
    #Initialize the node
    rospy.init_node('prm_planner', anonymous=False)
    
    #Generate a graph
    graph = learningPhrase(1000)
    
    #Create start and goal configurations
    start = generateSample()
    goal = generateSample()
    
    #Query the graph
    path = queryPhase(start, goal, graph)
    
    #Create a PRMPath message
    pathMSG = PRMPath()
    
    for node in path:
        # Create JointState object
        j = JointState()
        j.header.stamp = rospy.Time.now()
        j.name = ['joint1', 'joint2', 'joint3']
        j.position = node.value

        pathMSG.qs.append(j)

    pub = rospy.Publisher('/prm_path', PRMPath, queue_size=10)
    rospy.sleep(1)
    pub.publish(pathMSG)

    
if __name__ == "__main__":
    main()