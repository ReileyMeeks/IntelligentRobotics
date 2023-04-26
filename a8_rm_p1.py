#!/usr/bin/env python3

import numpy as np
import fcl 

#Createv fcl collision request and result objects
colRequest = fcl.CollisionRequest(enable_contact=True)
colResult = fcl.CollisionResult()

#Create fcl distance request and result objects
distRequest = fcl.DistanceRequest(enable_nearest_points=True)
distResult = fcl.DistanceResult()

def collisionPrint():
    #Print output of collision check
    print("Did Collide: ")
    print(colResult.is_collision)
    
def distancePrint():
    #Print output of distance check
    print("Distance Check: ")
    print(distResult.min_distance)

def boxOneTwo():
    #Create fcl box objects b1 and b2
    b1 = fcl.Box(1.0, 1.0, 1.0)
    b2 = fcl.Box(2.0, 2.0, 2.0)

    #Set position of b1 and b2
    b1Location = fcl.Transform(np.array([5.0, 5.0, 5.0]))
    b2Location = fcl.Transform(np.array([0.0, 0.0, 0.0]))

    #Check for collision between b1 and b2
    collisionCheck = fcl.collide(fcl.CollisionObject(b1, b1Location), fcl.CollisionObject(b2, b2Location), colRequest, colResult)
    
    #Print output of collision check
    collisionPrint()
    
    #Check for distance between b1 and b2
    distanceCheck = fcl.distance(fcl.CollisionObject(b1, b1Location), fcl.CollisionObject(b2, b2Location), distRequest, distResult)
    
    #Print the shortest path
    distancePrint()
    
    #Create new line
    print()
    
def boxThreeFour():
    #Create fcl box objects b3 and b4
    b3 = fcl.Box(2.0, 2.0, 2.0)
    b4 = fcl.Box(3.0, 3.0, 3.0)
    
    #Set position of b3 and b4
    b3Location = fcl.Transform(np.array([0.0, 0.0, 0.0]))
    b4Location = fcl.Transform(np.array([1.0, 1.0, 1.0]))
    
    #Check for collision between b3 and b4
    collisionCheck = fcl.collide(fcl.CollisionObject(b3, b3Location), fcl.CollisionObject(b4, b4Location), colRequest, colResult)
    
    #Print output of collision check
    collisionPrint()
    
    #Check for distance between b3 and b4
    distanceCheck = fcl.distance(fcl.CollisionObject(b3, b3Location), fcl.CollisionObject(b4, b4Location), distRequest, distResult)
    
    #Print the shortest path
    distancePrint()
    
    #Create new line
    print()
    
def boxFiveSix():
    #Create fcl box objects b5 and b6
    b5 = fcl.Box(2.0, 2.0, 2.0)
    b6 = fcl.Box(2.0, 2.0, 2.0)
    
    #Set position of b5 and b6
    b5Location = fcl.Transform(np.array([0.0, 0.0, 0.0]))
    b6Location = fcl.Transform(np.array([2.0, 0.0, 0.0]))
    
    #Check for collision between b5 and b6
    collisionCheck = fcl.collide(fcl.CollisionObject(b5, b5Location), fcl.CollisionObject(b6, b6Location), colRequest, colResult)
    
    #Print output of collision check
    collisionPrint()
    
    #Check for distance between b5 and b6
    distanceCheck = fcl.distance(fcl.CollisionObject(b5, b5Location), fcl.CollisionObject(b6, b6Location), distRequest, distResult)
    
    #Print the shortest path
    distancePrint()
    
    #Create new line
    print()
    
def main():
    #Me
    print('In main: <Reiley Meeks> <801149501> <rmeeks3@uncc.edu>')
    print()
    
    #Check for collision and distance between b1 and b2
    print("Collision Check of b1 and b2:")
    boxOneTwo()
    
    #Check for collision and distance between b3 and b4
    print("Collision Check of b3 and b4:")
    boxThreeFour()
    
    #Check for collision and distance between b5 and b6
    print("Collision Check of b5 and b6:")
    boxFiveSix()
    
    return 0
    
if __name__ == '__main__':
    main()
