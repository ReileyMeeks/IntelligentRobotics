#!/usr/bin/env python3
# TODO: Reiley Meeks
# TODO: rmeeeks3@uncc.edu 


import rospy
from tf2_ros import TransformBroadcaster
import math
import numpy as np
import tf_conversions
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R

joint_angles = [0.0,0.0,0.0]

def convertMatToTf_scipy(mat:np.array)-> TransformStamped:
    ''' Converts numpy matrix to a TransformStamped Object 
    '''
    result = TransformStamped()
    # Translation is set based on position vector in transformation matrix
    result.transform.translation.x = mat[0,3]
    result.transform.translation.y = mat[1,3]
    result.transform.translation.z = mat[2,3]
    rot_mat = mat[0:3,0:3]
    r = R.from_matrix(rot_mat)
    q = r.as_quat()
    # Set rotation
    result.transform.rotation.x = q[0]
    result.transform.rotation.y = q[1]
    result.transform.rotation.z = q[2]
    result.transform.rotation.w = q[3]
    return result

def getT(theta_i:float, a_iMinusOne:float, alpha_iMinusOne:float, d_i:float)->np.array:
    # TODO: calculate 4x4 transformation matrix for one set of DH parameters for one joint
    # Return the transformation matrix
    # Hint: Use the DH parameters to calculate the matrix
    # Hint: Use the numpy library to create the matrix
    calpha = np.cos(alpha_iMinusOne)
    ctheta = np.cos(theta_i)
    salpha = np.sin(alpha_iMinusOne)
    stheta = np.sin(theta_i)
    
    return np.array([
        [ctheta, -stheta, 0, a_iMinusOne],
        [stheta*calpha, ctheta*calpha, -salpha, -d_i*salpha],
        [stheta*salpha, ctheta*salpha, calpha, d_i*calpha],
        [0, 0, 0, 1]
    ])
    

def callback(data):
    ''' Callback function for joint angles
    '''
    global joint_angles
    joint_angles = data.position

def dh_parameters_to_mat(dh)->np.array: # TODO: Add list of DH parameters here
    ''' Calculates numpy array based on DH parameter list
    '''
    T01 = getT(dh["theta"][0], dh["a"][0], dh["alpha"][0], dh["d"][0])
    T12 = getT(dh["theta"][1], dh["a"][1], dh["alpha"][1], dh["d"][1])
    T23 = getT(dh["theta"][2], dh["a"][2], dh["alpha"][2], dh["d"][2])
    
    T03 = T01 @ T12 @ T23
    return T03

def main():
    global joint_angles
    
    # -- initialize node
    # TODO: Initialize node
    # Hint: Use rospy.init_node(<Fill details>)
    rospy.init_node('rm_a4', anonymous=True)
    
    # -- initialize subscriber
    rospy.Subscriber('joint_states', JointState, callback)

    # -- initialize transform broadcaster
    br = TransformBroadcaster()
    rospy.sleep(0.1)

    # -- initialize link lengths
    l1 = 3.0
    l2 = 3.0
    l3 = 2.0
    # TODO: Write down the DH Parameters here
    # Hint: Use variables that are easy to follow
    # Hint: Use the link lengths to calculate the DH parameters
    dh = {
        "joint": [1, 2, 3],
        "alpha": [0, np.pi/2, 0],
        "a": [0, 0, l1],
        "d": [0, 0, 0],
        "theta": joint_angles
    }
    
    # -- set up rate
    rate = rospy.Rate(10)
    
    # -- publish Transform stamped object
    while (rospy.is_shutdown() == False):
        # The joint angles are saved as joint_angles
        # TODO: Calculate the matrix using the joint angles and link lengths (l1, l2, and l3) as required.
        # Hint: Use the joint_angles within the while loop to get updated DH parameters for each joint
        # Hint: mat = dh_parameters_to_mat(...)
        dh.update({"theta": joint_angles})
        T03 = dh_parameters_to_mat(dh)

        # -- Calculate transformation matrix T_B_0 and T_3_E. 
        # Hint: You may need to use the code from geometric transformation assignment.
        # Hint: Get T_B_E = T_B_0 @ mat @ T_3_E
        TB0 = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, l1],
            [0, 0, 0, 1]
        ])
        
        T3E = np.array([
            [1, 0, 0, l3],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        TBE = TB0 @ T03 @ T3E
        
        # -- Convert the 4x4 matrix to a TransformStamped object
        tf_msg = convertMatToTf_scipy(TBE) # TODO: change mat to the matrix you calculated
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.child_frame_id = "rm" # TODO: Change this to your initials
        tf_msg.header.frame_id = "base_link"
        br.sendTransform(tf_msg)
        rate.sleep()
    
    print("Exit normally")

if __name__ == '__main__':
    main()

