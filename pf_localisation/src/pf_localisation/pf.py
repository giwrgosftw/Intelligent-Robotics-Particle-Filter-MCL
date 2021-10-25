from geometry_msgs.msg import Pose, PoseArray, Quaternion
from . pf_base import PFLocaliserBase
import math
import rospy

from . util import rotateQuaternion, getHeading
from random import random

from time import time


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
 
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        intX = initialpose.pose.pose.position.x 
        intY = initialpose.pose.pose.position.y
        intZ = initialpose.pose.pose.position.z 


        posArray = PoseArray()
 
        rotation = math.pi 


        #Creating 10 particles
        for i in range(0, 10):

            x  = random.gauss(intX, 1)
            y  = random.gauss(intY, 1)

            rotation = random.gauss(0, math.pi)

            pose = Pose()

            quat_tf = [0, 1, 0, 0]
            quat_tf[0] = x 
            quat_tf[1] = y 
            quat_tf[2] = intZ 

            quat_msg = Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])

            myrotateQuaternion = rotateQuaternion(quat_msg, rotation)
            
            # position of the particles
            pose.position.x  =  x 
            pose.position.y  = y 
            pose.position.z  = intZ

            pose.orientation =  myrotateQuaternion
            posArray.poses.append(pose)


        posArray.header.frame_id = "map"
   
        return posArray

 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        pass

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        pass
