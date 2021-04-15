#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math

from random import randint, random

# Likelihood field
from likelihood_field import LikelihoodField


def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation 
        Helper function from class 6 for use in measurement model"""
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample(elements, probabilities, n):
    """ Draws a random sample of n elements from a given list of choices 
        and their specified weights. Samples with replacement. """
    return np.random.choice(a=elements, size=n, replace=True, p=probabilities).tolist()


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map and likelihood field
        self.map = OccupancyGrid()
        self.lh_field = LikelihoodField()

        # the number of particles used in the particle filter
        self.num_particles = 10000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()


        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True



    def get_map(self, data):
        '''Called when map is first loaded'''
        self.map = data
        #print(data) for testing
    

    def initialize_particle_cloud(self):
        ''' Using self.map, get occupancy grid.
            In grid, get all cells that have a positive weight, normalize and random sample'''
        #drills to find all acceptable locations in occupancy grid
        spaces = []
        weights = []
        for row in range (self.map.info.width):
            for col in range (self.map.info.height):
                ind = row + col*self.map.info.width
                w = self.map.data[ind]
                if w > 0:
                    spaces.append((row, col))
        
        # draw_random_sample has to use 1-d array (so work with list indices 
        chosen_spaces = draw_random_sample(list(range(0, len(spaces))), [1.0 / len(spaces)] * len(spaces), self.num_particles)
        
        # Make particle objects
        particles = []
        for space in chosen_spaces:
            p = Pose()
            p.position.x = spaces[space][0]
            p.position.y = spaces[space][1]
            random_yaw = randint(0, 359)
            q = quaternion_from_euler(0, 0, random_yaw)
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            new_particle = Particle(p, 1.0) 
            self.particle_cloud.append(new_particle)
        self.normalize_particles()

        self.publish_particle_cloud()
        print("init done")


    def normalize_particles(self):
        ''' make all the particle weights sum to 1.0
            Calculate total particle weight sum'''
        weight_sum = 0.0
        for p in self.particle_cloud:
            weight_sum += p.w
        for p in self.particle_cloud:
            p.w /= weight_sum
        


    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):
        # TODO
        particle_poses = [p.pose for p in self.particle_cloud] # get list of particle poses
        weights = [p.w for p in self.particle_cloud] # get probabilities (weights) for all particle poses
        new_sample = draw_random_sample(particle_poses, weights, self.num_particles) # random sample
        self.particle_cloud = new_sample



    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model(curr_x - old_x, curr_y - old_y, curr_yaw - old_yaw)

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        # This function takes the normalized weighted average of all particles 
        # https://en.wikipedia.org/wiki/Weighted_arithmetic_mean
        # TODO
        x_avg = 0
        y_avg = 0
        quaternion_x_avg = 0
        quaternion_y_avg = 0
        quaternion_z_avg = 0
        quaternion_w_avg = 0

        for p in self.particle_cloud:
            x_avg += (p.w * p.pose.position.x)
            y_avg += (p.w * p.pose.position.y)
            quaternion_x_avg += (p.w * p.pose.orientation.x)
            quaternion_y_avg += (p.w * p.pose.orientation.y)
            quaternion_z_avg += (p.w * p.pose.orientation.z)
            quaternion_w_avg += (p.w * p.pose.orientation.w)
        new_pose = Pose()
        new_pose.position.x = x_avg
        new_pose.position.y = y_avg
        new_pose.orientation.x = quaternion_x_avg
        new_pose.orientation.y = quaternion_y_avg
        new_pose.orientation.z = quaternion_z_avg
        new_pose.orientation.w = quaternion_w_avg


    
    def update_particle_weights_with_measurement_model(self, data):
        # Update with likelihood field 
        # TODO
        cardinal_direction_idxs = [0, 45, 90, 135, 180, 225, 270, 315]
        lidar_measurements = [data.ranges[i] for i in cardinal_direction_idxs] # I think this works in python but test

        # Iterate through every particle
        for particle in self.particle_cloud:
            q = 1
            for i in range(len(cardinal_direction_idxs)):
                if measurements[i] <= 3.5:
                    euler_angle = get_yaw_from_pose(particle.pose)
                    adjusted_x = particle.pose.position.x + (lidar_measurements[i] * math.cos(euler_angle + math.radians(cardinal_direction_idxs[i])))
                    adjusted_y = particle.pose.position.y + (lidar_measurements[i] * math.sin(euler_angle + math.radians(cardinal_direction_idxs[i])))
                    dist = self.likelihood_field.get_closest_obstacle_distance(adjusted_x, adjusted_y)
                    q = q * (compute_prob_zero_centered_gaussian(dist, 0.1)) # adjust this to be the more complicated version
            particle.w = q

    
    def generate_noise(self, center, scale):
        """This helper function generates random noise based on a normal distribution
            center: center of the distribution, scale: 'width' of distribution (standard deviation)"""
        return random_sample.normal(loc=center, scale=scale)
    

    def update_particles_with_motion_model(self, x_diff, y_diff, yaw_diff):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly
        # Generate noise with generate_noise helper function
        # TODO
        for particle in self.particle_cloud:
            particle_x = particle.pose.position.x
            particle_y = particle.pose.position.y
            particle_theta = get_yaw_from_pose(particle.pose)
            # Can play with scale of noise later
            particle.pose.position.x = self.generate_noise(particle_x + x_diff, 0.05)
            particle.pose.position.y = self.generate_noise(particle_y + y_diff, 0.05)
            new_theta = self.generate_noise(particle_theta + yaw_diff, 0.05) % 360 # I think this makes angle from 0-359
            particle.pose.orientation = quaternion_from_euler([0, 0, new_theta])


if __name__=="__main__":
    
    pf = ParticleFilter()

    rospy.spin()


