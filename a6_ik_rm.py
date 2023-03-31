#!/usr/bin/env python3

import rospy
import tf_conversions
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from math import modf, sin,cos,atan2,pi,fmod,fabs
from numpy import mat, array, transpose
import numpy as np
from numpy.linalg import norm
import sympy as sy
from sympy import symbols, diff

# STUDENT: FILL IN YOUR DETAILS HERE
__author__ = "Reiley Meeks"
__email__ = "rmeeks3@uncc.edu"
__studentID__ = "801149501"
__date__ = "30 March 2023"

l1=3.0
l2=3.0
l3=2.0

pub_joints = rospy.Publisher('joint_states', JointState, queue_size=10)


def pubJointState(thetas):
    '''
	Converts a list of joint values to a JointState object, and published the object to topic "joint_states"
    Parameters:
        thetas (list): a list with three joint values

    Returns:
        None
    '''

    # Create JointState object
    j = JointState()
    j.header.stamp = rospy.Time.now()
    j.name = ['joint1', 'joint2', 'joint3']
    j.position = thetas
    j.velocity = []
    j.effort = []

    # Publish it
    pub_joints.publish(j)


def findDistanceBetweenAngles(a:float, b:float)->float:
    '''
    Get the smallest orientation difference in range [-pi,pi] between two angles
    Parameters:
        a (double): an angle
        b (double): an angle

    Returns:
        double: smallest orientation difference in range [-pi,pi]
    '''
    result = 0
    difference = b - a
   
    if difference > pi:
      difference = fmod(difference, pi)
      result = difference - pi
 
    elif(difference < -pi):
      result = difference + (2*pi)
 
    else:
      result = difference
 
    return result


def displaceAngle(a1:float, a2:float)->float:
    '''
    Displace an orientation by an angle and stay within [-pi,pi] range
    Parameters:
        a1 (double): an angle
        a2 (double): an angle

    Returns:
        double: The resulting angle in range [-pi,pi] after displacing a1 by a2
    '''
    a2 = a2 % (2.0*pi)
 
    if a2 > pi:
        a2 = (a2 % pi) - pi
 
    return findDistanceBetweenAngles(-a1,a2)


def displaceAngleVec(a1:np.ndarray, a2:np.ndarray)->np.ndarray:
    ''' 
    Calculate the addition of two sets of joint angles
    The resultant vector will be in range [-pi,pi]

    Parameters:
        a1 (numpy.ndarray): a vector of angles
        a2 (numpy.ndarray): a vector of angles
    
    Returns:
        result (numpy.ndarray): a vector of angles in range [-pi,pi]
    '''
    result = [0 for i in range(len(a1))]
    for i in range(len(a1)):
        a2[i] = a2[i] % (2.0*pi)
    
        if a2[i] > pi:
            a2[i] = (a2[i] % pi) - pi
 
        result[i] = findDistanceBetweenAngles(-a1[i],a2[i])

    return array(result)


def FK(theta1, theta2, theta3):
    ''' Calculate the forward kinematics for the given joint angles
    '''
    r = l2 * cos(theta2) + l3 * cos(theta2 + theta3)
    x = r * cos(theta1)
    y = r * sin(theta1)
    z = l1 + (l2 * sin(theta2)) + (l3 * sin(theta2 + theta3))
    return np.array([x, y, z])


def getJInv(theta1:float,theta2:float,theta3:float)->np.ndarray:
    ''' 
    Calculates the 3x3 Jacobian inverse matrix for the given joint angles
    Parameters:
        theta1 (float): joint 1 angle
        theta2 (float): joint 2 angle
        theta3 (float): joint 3 angle

    Returns:
        J_inv (numpy.ndarray): 3x3 Jacobian inverse matrix
    '''
    J = np.zeros((3,3), dtype=np.float64)
    r = (l2*cos(theta2)) + (l3*cos(theta2+theta3))
    
    # -- populate the Jacobian matrix
    # STUDENT: Implement the Jacobian inverse calculation here
    #x
    dx_dtheta1 = r * -sin(theta1)
    dx_dtheta2 = -l2 * cos(theta1) * sin(theta2) - l3 * cos(theta1) * sin(theta2 + theta3)
    dx_dtheta3 = -l3 * cos(theta1) * sin(theta2 + theta3)
    
    #y
    dy_dtheta1 = l2 * cos(theta2) * cos(theta1) + l3 * cos(theta2 + theta3) * cos(theta1)
    dy_dtheta2 = -l2 * sin(theta1) * sin(theta2) - l3 * sin(theta1) * sin(theta2 + theta3)
    dy_dtheta3 = -l3 * sin(theta1) * sin(theta2 + theta3)
    
    #z
    dz_dtheta1 = 0
    dz_dtheta2 = l2 * cos(theta2) + l3 * cos(theta2 + theta3)
    dz_dtheta3 = l3 * cos(theta2 + theta3)
    
    J = np.array([[dx_dtheta1, dx_dtheta2, dx_dtheta3], 
                  [dy_dtheta1, dy_dtheta2, dy_dtheta3], 
                  [dz_dtheta1, dz_dtheta2, dz_dtheta3]])
    

    J_inv = np.linalg.pinv(J) # returns the Moore-Penrose Pseudoinverse 
    return J_inv


def IK(eePose):
    '''
    Calculate the inverse kinematics for the given end effector pose.
    Uses Newton Raphson Method to calculate the joint angles

    Parameters:
        eePose (geometry_msgs.msg.Pose): end effector pose
    
    Returns:
        Three joint angles as a list
    '''
    # STUDENT: Implement the numerical inverse kinematics algorithm here
    #Set initial joint angles to 0
    thetaI = np.array([0, 0, 0])
  
    #Calculate the forward kinematics, based on the initial joint angles estimate
    fk = FK(thetaI[0], thetaI[1], thetaI[2])
  
    #Initialize the error and counter
    i = 0
    error = 0.001
  
    #Current end effector position
    xd = np.array([eePose.position.x, eePose.position.y, eePose.position.z])
  
    #Calculate e
    e = xd - fk
  
    #Start while loop
    while np.linalg.norm(e) > error and i < 1000:
        ji = getJInv(thetaI[0], thetaI[1], thetaI[2])
        #Calculate the new joint angles
        thetaI = thetaI + ji @ e
      
        #Calculate the forward kinematics, based on the new joint angles estimate
        fk = FK(thetaI[0], thetaI[1], thetaI[2])
      
        #Calculate e
        e = xd - fk
      
        if np.linalg.norm(e) < error:
            print("Loop ran: ", i, " times")
            break
      
      
        #Increment counter
        i += 1
      
    #Return the joint angles
    if np.linalg.norm(e) > error:
        return [None, None, None]
    
    for i in range(len(thetaI)):
        thetaI[i] = thetaI[i] % (2.0*pi)
    
        if thetaI[i] > pi:
            thetaI[i] = (thetaI[i] % pi) - pi
            
    return thetaI
    
    


def main():
    # STUDENT: Modify print statement below
    print('In main: <Reiley Meeks> <801149501> <rmeeks3@uncc.edu>')

    # Initialize the node
    rospy.init_node('spatial3r_ik_newtons', anonymous=False)
    rospy.sleep(0.5)
    
    #Test case end effector position
    #Last test fails
    testVals = [[5.0, 0.0, 3.0], #Worked 0
                [3.0, 0.0, 5.0], #Worked 1
                [2.6481, 1.5281, 6.8881], #Worked 2
                [2.9131, -0.1704, 0.8434], #Worked 3
                [-0.0255, 1.8448, 4.8256], #Did not work 4
                [10, 0, 3]] #Worked 5

    # Test case end effector position
    p = Pose()
    for i in range(0, len(testVals)):
        print(f"Test case {i + 1}:")
        p.position.x = testVals[i][0]
        p.position.y = testVals[i][1]
        p.position.z = testVals[i][2]
        
        print("XPos: " + str(p.position.x))
        print("YPos: " + str(p.position.y))
        print("ZPos: " + str(p.position.z))
    
        solution = IK(p)

        if solution[0] == None:
            print("No solution found")
            print("")
        else:
            print(f"Solution found: [{solution[0]:.4f}, {solution[1]:.4f}, {solution[2]:.4f}]")
            print("")
            pubJointState(solution)

    print("\nExiting normally")

if __name__ == "__main__":
    main()
