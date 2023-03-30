#!/usr/bin/env python3
import rospy
import tf_conversions
import tf2_ros
import numpy as np
from math import sin, cos, atan2, sqrt, pow, acos, pi, fmod
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose

__author__ = "Reiley Meeks"
__email__ = "rmeeks3@uncc.edu"
__studentID__ = "801149501"
__date__ = "17 March 2023"

l1 = 3.0
l2 = 3.0
l3 = 2.0

# Publisher to use in the pubJointState function
pub_joints = rospy.Publisher("joint_states", JointState, queue_size=10)


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


def findDistanceBetweenAngles(a, b):
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


def displaceAngle(a1, a2):
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


def solveTheta2And3(r:float, z:float)->list:
    '''
    Solves for theta 2 and 3. There are two solutions.

    Parameters:
        r (double): signed length of vector from the origin to the end-effector position on the xy plane
        z (double): z value of the end-effector position

    Returns:
        list: 2D list in the form [[theta2_a, theta3_a], [theta2_b, theta3_b]]
    '''
    # STUDENT
    # Check if solution exists before calculating the solutions
    
    #Calc d
    d = (r**2 + (z - l1)**2 - l2**2 - l3**2) / (2 * l2 * l3)
    
    #Check if d is > 1, if yes fail
    if np.abs(d) > 1:
        print("This is not a valid solution!")
        return None
    
    
    #Calc Theta3
    theta3_a = np.arctan2(np.sqrt(1 - d**2), d)
    theta3_b = np.arctan2(-np.sqrt(1 - d**2), d)
    
    #Calc psi, and the 2 phi values
    psi = np.arctan2(z - l1, r)
    phiA = np.arctan2(l3 * np.sin(theta3_a), l2 + l3 * np.cos(theta3_a))
    phiB = np.arctan2(l3 * np.sin(theta3_b), l2 + l3 * np.cos(theta3_b))
    
    #Calc theta2
    theta2_a = psi - phi_a
    theta2_b = psi - phi_b
    
    #Debug Values
    print("Debug Start")
    print("theta2_a: ", theta2_a)
    print("theta3_a: ", theta3_a)
    print("theta2_b: ", theta2_b)
    print("theta3_b: ", theta3_b)
    print("Debug End")
    #End Debug Values
    
    return [[theta2_a, theta3_a], [theta2_b, theta3_b]]


def IK(eePose):
    '''
    Computes IK solutions for a given end-effector pose.

    Parameters:
        eePose (Pose): the pose of the end-effector

    Returns:
        list: The 4 solutions for IK, or None if no solution exists
    '''
    print("In IK")
    x = eePose.position.x
    y = eePose.position.y
    z = eePose.position.z

    # STUDENT
    # Solve for theta 1 below. There are two solutions.
    #******************************************************
    # theta1
    # theta 1 is based on an overhead view.
    # Hint: There would be two values of theta1 (theta1A and theta1B)
    #******************************************************
    
    theta1A = np.arctan2(y, x)
    theta1B = np.arctan2(-y, -x)
    
    r = np.sqrt(x**2 + y**2)


    # STUDENT Uncomment the lines below and fill in the parameters
    solsRA = solveTheta2And3(r=r, z=z) # for theta1A
    solsRB = solveTheta2And3(r=-r, z=z) # for theta1B


    # STUDENT
    # Create code that creates the list of solutions (if solutions exist) and return it
    # Return None if no solutions exist
    if len(solsRA) > 1:
        # Hint: use theta1A with solsRA and theta1B with solsRB
        sol1 = [theta1A, solsRA[1][0], solsRA[1][1]]
        sol2 = [theta1A, solsRA[0][0], solsRA[0][1]]
        sol3 = [theta1B, solsRB[1][0], solsRB[1][1]]
        sol4 = [theta1B, solsRB[0][0], solsRB[0][1]]
        return [sol1, sol2, sol3, sol4]
    else:
        return None




def main():
    # STUDENT: Modify print statement below
    print('In main: <Reiley Meeks> <801149501> <rmeeks3@uncc.edu>')

    rospy.init_node('spatial3r_ik', anonymous=False)
    rospy.sleep(0.5)
    
    #Test values, row 1, 2, 3 all pass, row 4 fails
    testVals = [[2.6481, 1.5281, 6.8881], 
                [2.9132, -0.1704, 0.8434], 
                [-0.0255, 1.8448, 4.8256], 
                [10, 0, 3]]


    # Test case end effector position
    p = Pose()
    
    p.position.x = testVals[0][0]
    p.position.y = testVals[0][1]
    p.position.z = testVals[0][2]

    sols = IK(p)
    for i,s in enumerate(sols):
        print('Solution %s: %s' % (i,s))

    # If solutions exist, then publish the joint states in Rviz
    if len(sols) > 0:
        # STUDENT
        # Choose one of the solutions from the list to visualize
        q = sols[0]

        # Publish the joint values to RViz
        pubJointState(q)

        # Check position with FK
        #x = cos(q[0]) * (l2 * cos(q[1]) + l3*cos(q[1]+q[2]))
        #y = sin(q[0]) * (l2 * cos(q[1]) + l3*cos(q[1]+q[2]))
        #z = l1 + l2*sin(q[1]) + l3*sin(q[1]+q[2])
        #print('FK: [%s, %s, %s]' % (x, y, z))


    print('Exiting normally')




if __name__ == "__main__":
    main()
