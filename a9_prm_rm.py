#!/usr/bin/env python3

# -- python imports
import random
import numpy as np
import copy
from math import pi, sin, cos, acos, atan2, fmod, sqrt, radians
import networkx as nx
import rospy
from tqdm import tqdm

# -- message import
from a7_prm.msg import PRMPath
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# Collision detection
import fcl
from tf_prac import translate, rotateY, rotateZ
from fcl_scaffold import *


qCurrent = [0.0,0.0,0.0]
global obs



'''
Step 1. Generate sample configurations and publish them to Rviz.
'''

def generateSample():
    # STUDENT TODO: Implement the sample generation
    # Return a sample configuration for a 3R manipulator
    # The sample should be a numpy array of size 3
    #Create N random samples, wrap them in a node object, and add them to the graph    
    #Define the range of joint angles
    theta1 = np.random.uniform(-np.pi, np.pi)
    theta2 = np.random.uniform(-np.pi, np.pi)
    theta3 = np.random.uniform(-np.pi, np.pi)
    
    return np.array([theta1, theta2, theta3])

'''
Step 2. "Learning phase" - Build a graph for the nodes.
- Collect samples in a list of nodes
- Check distance threshold to get nearby nodes
- Use local planner to check if there is an edge between nodes
'''

class Node:
    def __init__(self, value):
        self.value = value
        
G = nx.Graph()

def collQuery(q, boxOb):
    request = fcl.CollisionRequest()
    result = fcl.CollisionResult()
    rLinks = getRobotCollOb(q)
    # Do collision for each link
    for l in rLinks:
        ret = fcl.collide(boxOb.collObj, l, request, result)
        if ret > 0:
            return True
    return False


def localPlanner(qa, qb):
    # STUDENT TODO: Implement the local planner
    # Return True if there is a collision-free path between qa and qb
    # Return False if there is a collision along the path
    # Part 2.1.c
    '''
    This function should interpolate a straight line path between two nodes (i.e., in the configuration space)
    and check for collisions at intermediate points along the straight-line path. 
    If there is no collision, then this function returns True. Otherwise, it returns False
    Recall that for any two configurations and, the ith intermediate q1q2 i Configuration on a straight line between them can be represented 
    As q' = q1 + ∆q * i, ∆q = (q2 - q1)/n + 1
    N = the number of intermediate steps between two configurations
    i = 1, 2, ..., N
    '''
    n = len(obs)
    qPrime = (qb - qa)/n+1
    path = [qa + i * qPrime for i in range(1, n+1)]
    
    for q in path:
        for o in obs:
            if collQuery(q, o):
                return False
    
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
    #Set threshold to 1.5708 radians
    threshold = 1.5708
    
    #Condition One, take abs value of the distance between angles
    fd = find_distance_between_angles
    getDst1 = abs(fd(node1.value[0], node2.value[0]))
    getDst2 = abs(fd(node1.value[1], node2.value[1]))
    getDst3 = abs(fd(node1.value[2], node2.value[2]))
    
    #print('Distances: ', distances)
    
    S = getDst1 + getDst2 + getDst3
    
    #Condition Two
    lp = localPlanner(node1.value, node2.value)
    
    if getDst1 < threshold and getDst2 < threshold and getDst3 < threshold and lp:
        #print('Edge exists between: ', node1.value, ' and ', node2.value)
        return S
    else:
        return -1 


def learningPhase(N):
    # STUDENT TODO: Implement the learning phase
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

'''
Step 3. "Query phase" - do graph search over the graph
'''
# Find node to connect start or goal node to
# Returns true or false based on if the node can be connected
def connectToGraph(node, graph):
    pass  # Delete this line when you implement the function


def queryPhase(start, goal, g):
    # STUDENT TODO: Implement the query phase
    # Part 3.2
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
    for node in tqdm(shortestPath):
        print(node.value)
    return shortestPath

def checkStartGoal(start, goal):
    startNode = Node(start)
    goalNode = Node(goal)
    
    for i in obs:
        if collQuery(startNode.value, i) or collQuery(goalNode.value, i):
            return False
    return True
    


def main():
    global obs
    rospy.init_node('PRM', anonymous=False)
    pub_rviz = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.sleep(0.5)
    
    #NOTE: Uncomment any one of the obstacle sets to test

    # -- create obstacles and publish them to rviz
    # create start and goal positions
    # -- obstacle set 1
    numObs = 1
    info = [[-1,-1,4,1,1,1]]
    start = np.array([0.0, 0.0, 0.0])
    goal = np.array([2*pi/3, 2*pi/3, 2*pi/3])
    obs = createObs(numObs, pub_rviz, rand=False, info=info)

    # -- obstacle set 2
    #numObs = 2
    #info = [[1,1,2,0.5,1.0,0.5],
            #[4,-3,3,0.8,0.4,0.33]]
    #start = np.array([0.0, 0.0, 0.0])
    #goal = np.array([2*pi/3, 2*pi/3, 2*pi/3])
    #obs = createObs(numObs, pub_rviz, rand=False, info=info)

    # -- obstacle set 3
    # numObs = 3
    # info = [[1,1,2,0.5,1.0,0.5],
    #         [-1,-1,4,1,1,1],
    #         [4,-3,3,0.8,0.4,0.33]]
    # start = np.array([0.0, 0.0, 0.0])
    # goal = np.array([2*pi/3, 2*pi/3, 2*pi/3])
    # obs = createObs(numObs, pub_rviz, rand=False, info=info)

    # -- random obstacle set
    # numObs = 3
    # obs = createObs(numObs, pub_rviz, rand=True)
    # start = generateSample()
    # goal = generateSample()
    

    # STUDENT TODO: Check for collision free start and goal configurations
    # Part 3.1
    
    if checkStartGoal(start, goal) == False:
        print("Start and goal configurations are not collision free")
        return
        


    # -- learning phase with 50 nodes
    graph = learningPhase(50)

    # -- print the total number of graph nodes and edges
    print(f"Number of graph nodes: {len(graph.nodes)}")
    print(f"Number of graph edges: {len(graph.edges)}")
    
    path = queryPhase(start, goal, graph)

    # STUDENT TODO: Print configurations in path
    # Part 3.3
    #Done in queryPhase()

    # STUDENT TODO: Uncomment to publish path in RViz

    #Create a PRMPath message
    pathMSG = PRMPath()
    
    for node in path:
        # Create JointState object
        j = JointState()
        j.header.stamp = rospy.Time.now()
        j.name = ['joint1', 'joint2', 'joint3']
        j.position = node.value

        pathMSG.qs.append(j)

    # STUDENT TODO: Create a publisher for the path
    # Part 4.4
    pub = rospy.Publisher('/prm_path', PRMPath, queue_size=10)
    rospy.sleep(1)
    pub.publish(pathMSG)
    

if __name__ == '__main__':
    main()
