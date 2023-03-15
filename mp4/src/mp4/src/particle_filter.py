import numpy as np
from maze import Maze, Particle, Robot
import bisect
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
import shutil
from std_msgs.msg import Float32MultiArray
from scipy.integrate import ode

def func1(t, vars, vr, delta):
    curr_x = vars[0]
    curr_y = vars[1]
    curr_theta = vars[2]

    dx = vr * np.cos(curr_theta)
    dy = vr * np.sin(curr_theta)
    dtheta = delta
    return [dx,dy,dtheta]

class particleFilter:
    def __init__(self, gem, world, grid_width, grid_height, num_particles, sensor_limit, kernel_sigma, x_start, y_start):
        self.num_particles = num_particles  # The number of particles for the particle filter
        self.sensor_limit = sensor_limit    # The sensor limit of the sensor
        self.rand_particle_ratio = 0.1
        particles = list()
        for i in range(num_particles):
            x = np.random.uniform(0, world.width)
            y = np.random.uniform(0, world.height)
            particles.append(Particle(x = x, y = y, maze = world, sensor_limit = sensor_limit))
        self.particles = particles          # Randomly assign particles at the begining
        self.bob = gem                      # The actual state of the vehicle
        self.world = world                  # The map of the maze
        self.grid_width = grid_width        # Each grid in gazebo simulator is divided into grid_width sections horizontally
        self.grid_height = grid_height      # Each grid in gazebo simulator is divided into grid_height sections vertically
        self.x_start = x_start              # The starting position of the map in the gazebo simulator
        self.y_start = y_start              # The starting position of the map in the gazebo simulator
        self.kernel_sigma = kernel_sigma
        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.controlSub = rospy.Subscriber("/gem/control", Float32MultiArray, self.__controlHandler, queue_size = 1)
        self.control = []                   # A list of control signal from the vehicle
        return

    def __controlHandler(self,data):
        tmp = list(data.data)
        self.control.append(tmp)
		
    def getModelState(self):
        """
        Description:
            Requests the current state of the polaris model when called
        Returns:
            modelState: contains the current model state of the polaris vehicle in gazebo
        """
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='polaris')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState

    def weight_gaussian_kernel(self,x1, x2, std = 10):
        """
        Description:
            Given the sensor reading from vehicle and the sensor reading from particle, compute the weight of the particle based on gaussian kernel
        Input:
            x1: The sensor reading from vehicle
            x2: The sensor reading from particle
        Returns:
            Returns weight of the particle
        """
        distance = np.linalg.norm(np.asarray(x1) - np.asarray(x2))
        return np.exp(-distance ** 2 / (2 * std))

    def showMarker(self, x, y):
        """
        Description:
            Update the position of the marker in gazebo environment
        Input:
            x: x position of the marker
            y: y position of the marker
        Returns:
        """
        markerState = ModelState()
        markerState.model_name = 'marker'
        markerState.pose.position.x = x/self.grid_width + self.x_start - 100
        markerState.pose.position.y = y/self.grid_height + self.y_start - 100
        self.modelStatePub.publish(markerState)

    def quaternion_to_euler(self, x, y, z, w):
        """
        Description:
            converts quaternion angles to euler angles. Note: Gazebo reports angles in quaternion format
        Inputs:
            x,y,z,w:
                Quaternion orientation values
        Returns:
            List containing the conversion from quaternion to euler [roll, pitch, yaw]
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        return [roll, pitch, yaw]

    def updateWeight(self, readings_robot):

        ## TODO #####
        # Update the weight of each particle based on sensor reading from the robot
        # Normalize the weight

        ###############
        return

    def resampleParticle(self):

        particles_new = list()

        ## TODO #####
        # Resample particles base on their calculated weight
        # Note: At least TWO different resampling method need to be implemented; You may add new function for other resampling techniques

        ###############

        self.particles = particles_new

    def particleMotionModel(self):

        ## TODO #####
        # Update the position of each particle based on the control signal from the vehicle


        ###############
        return 

    def runFilter(self):
        # Run PF localization
        while True:
            ## TODO #####
            # Finish this function to have the particle filter running

            # Read sensor msg

            # Display actual and estimated state of the vehicle and particles on map
            self.world.show_particles(particles = self.particles, show_frequency = 10)
            self.world.show_robot(robot = self.bob)
            [est_x,est_y] = self.world.show_estimated_location(particles = self.particles)
            self.world.clear_objects()
            self.showMarker(est_x, est_y)

            # Resample particles
