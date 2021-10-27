from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from .pf_base import PFLocaliserBase
import math
from .util import rotateQuaternion, getHeading
import random


class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()

        # ----- Set motion model parameters
        self.num_poses = 500

        # Need for the "not_update_cloud" function
        self.latest_odom_x = 0
        self.latest_odom_y = 0
        self.latest_odom_heading = 0

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20  # Number of readings to predict

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

        # Set initial poses parameters
        intX = initialpose.pose.pose.position.x
        intY = initialpose.pose.pose.position.y

        # Set an array with pose objects
        poseArray = PoseArray()

        # For each particle/pose
        for i in range(self.num_poses):
            # Set random floating point number with gaussian distribution
            # from initial x,y position as mean
            x = random.gauss(intX, 1)  # mean # sigma
            y = random.gauss(intY, 1)

            # Set an Pose object
            pose = Pose()

            # Set random position to the Pose
            pose.position.x = x
            pose.position.y = y

            # Set rotation by Math.PI radians
            rotation = random.gauss(0, math.pi)

            # Create a list of floats, which is compatible with tf quaternion methods
            # Source: http://wiki.ros.org/tf2/Tutorials/Quaternions
            quat_tf = [0, 1, 0, 0]
            quat_tf[0] = x
            quat_tf[1] = y

            quat_msg = Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])

            # Set rotateQuaternion based on the above
            myRotateQuaternion = rotateQuaternion(quat_msg, rotation)

            # Insert the resulting Quaternion back into the Pose
            # Now, we are expecting that each of our initial particles to have different direction
            pose.orientation = myRotateQuaternion

            # Adding the final Pose (with the new orientation) into the Array pose
            poseArray.poses.append(pose)

        return poseArray

    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.

        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        # --- Initially do not update the particles if no movement has been detected

        # Keep the latest odom as the previous one
        prev_odom_list = [
            self.prev_odom_x == self.latest_odom_x,
            self.prev_odom_y == self.latest_odom_y,
            self.prev_odom_heading == self.latest_odom_heading
        ]

        # If: all are True (not 0), do nothing (the robot can move)
        if all(prev_odom_list):
            return

        # Else: the previous odom is still the latest one (so, do not move)
        self.latest_odom_x = self.prev_odom_x
        self.latest_odom_y = self.prev_odom_y
        self.latest_odom_heading = self.prev_odom_heading

        # ---------------------------------------------------------------

        # Initializing array of particles/poses
        pose = self.particlecloud

        # Create a list of weights
        weights = []

        # Compute the likelihood weight for each particle
        for i, p in enumerate(pose.poses):
            # get the likelihood weighting
            likhweight = self.sensor_model.get_weight(scan, p)  # laser scan and position
            weights.append(likhweight)  # add that weight into the list

        total = sum(weights)  # calculate the sum of the weights

        # Normalize weights of the "weight" list
        # (total contribution sums up to 1 - decrease variance - higher accuracy in prediction)
        norm = [w / total for w in weights]

        # Total number of particles
        M = self.num_poses

        # Generate the Cumulative distribution function
        # using the above normalized weights
        cdf = []
        for i, n in enumerate(norm):
            if i == 0:
                cdf.append(n)  # c[0] = w[1]
            else:
                cdf.append(cdf[-1] + n)  # c[i] = c[i-1] + w

        # Resampling
        thold = 1 / M  # threshold = M^(-1)
        u = random.uniform(0, thold)  # distribution between 0 and threshold
        i = 0  # for the while loop
        poseArray = PoseArray()  # Setting a new array of poses/particles

        # For each particle
        for _ in range(M):

            # Every particle with normalized weight over 1/N is guaranteed to be selected at least once
            # So, skip until next threshold reached
            while u > cdf[i]:
                i = i + 1

            # Getting the existing selected particle from the cloud
            particles = self.particlecloud.poses[i]

            # Take its position and orientation
            x = particles.position.x
            y = particles.position.y
            q = particles.orientation
            t = getHeading(q)  # Get heading (in radians) described by a given orientation

            # Set a new random location proportional to the normalized weight of the existing-selected particle
            # (random floating point number with gaussian distribution)
            rx = random.gauss(x, norm[i])
            ry = random.gauss(y, norm[i])
            rt = random.gauss(t, norm[i])

            # Using the above, create a new pose with a new location and orientation (resampling applies)
            rPoint = Point(rx, ry, 0.0)  # z usually is 0
            rotateQ = rotateQuaternion(q, rt - t)
            newPose = Pose(rPoint, rotateQ)

            # Increment threshold
            u = u + thold

            # Add the new pose
            poseArray.poses.append(newPose)

        # Update/Replace the existing particles/poses with the new ones on the cloud
        self.particlecloud = poseArray

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

        # Take a list with all current particles
        particles = self.particlecloud.poses

        # Set a list to store the sorted particles
        particles_sorted = []

        # For each particle make a 2D list
        # First element: is the index of the particular particle
        # Second element: the weight of that particle
        for i in range(0, len(particles)):
            particle_w = particles[i].orientation.w
            particles_sorted.append([i, particle_w])

        # Sort the 2D array based on their weight (higher ones on top)
        particles_sorted = sorted(particles_sorted, key=lambda x: x[1], reverse=True)

        # Take the first column of the 2D list
        # The order of the indexes of the particles sorted by weight
        indexs = [i[0] for i in particles_sorted]

        # Selecting the top 50% of the particles based one the weight (best particles)
        best_particles = [particles[indexs[i]] for i in range(0, len(indexs)//2)]

        # Particle Coordinates - used for the estimate pose
        xpos = []
        ypos = []

        # Particle Headings - used for estimate pose (orientation)
        xor = []
        yor = []
        zor = []
        wor = []

        # Extracting the coordinates for each best particle
        for particle in best_particles:
            xpos.append(particle.position.x)
            ypos.append(particle.position.y)
            xor.append(particle.orientation.x)
            yor.append(particle.orientation.y)
            zor.append(particle.orientation.z)
            wor.append(particle.orientation.w)

        # Calculating the average pose
        meanxpos = sum(xpos) / len(xpos)
        meanypos = sum(ypos) / len(ypos)
        meanxor = sum(xor) / len(xor)
        meanyor = sum(yor) / len(yor)
        meanzor = sum(zor) / len(zor)
        meanwor = sum(wor) / len(wor)

        # Putting the estimates pose values into an Pose Object
        est_pose = Pose()

        est_pose.position.x = meanxpos
        est_pose.position.y = meanypos
        est_pose.position.z = 0.0
        est_pose.orientation.x = meanxor
        est_pose.orientation.y = meanyor
        est_pose.orientation.z = meanzor
        est_pose.orientation.w = meanwor

        # Return the estimate pose
        return est_pose
