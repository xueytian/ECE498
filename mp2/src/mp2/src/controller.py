import math
import numpy as np

import rospy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive




class bicycleModel():

    def __init__(self):

        """ Series of provided waypoints """
        pt1 = ModelState()
        pt2 = ModelState()
        pt3 = ModelState()
        pt4 = ModelState()
        pt5 = ModelState()
        pt6 = ModelState()
        pt7 = ModelState()

        pt1.pose.position.x = 10
        pt1.pose.position.y = 0
        pt1.twist.linear.x = .25
        pt1.twist.linear.y = .25

        pt2.pose.position.x = 10
        pt2.pose.position.y = 10
        pt2.twist.linear.x = .25
        pt2.twist.linear.y = .25

        pt3.pose.position.x = -10
        pt3.pose.position.y = -10
        pt3.twist.linear.x = .25
        pt3.twist.linear.y = .25

        pt4.pose.position.x = 10
        pt4.pose.position.y = -10
        pt4.twist.linear.x = .25
        pt4.twist.linear.y = .25

        pt5.pose.position.x = -10
        pt5.pose.position.y = -10
        pt5.twist.linear.x = .25
        pt5.twist.linear.y = .25

        pt6.pose.position.x = -10
        pt6.pose.position.y = 0
        pt6.twist.linear.x = .25
        pt6.twist.linear.y = .25

        pt7.pose.position.x = 0
        pt7.pose.position.y = 0
        pt7.twist.linear.x = .25
        pt7.twist.linear.y = .25


        self.waypointList = [pt1, pt2, pt3, pt4, pt5, pt6, pt7]

        self.length = 1.88

        self.waypointSub = rospy.Subscriber("/gem/waypoint", ModelState, self.__waypointHandler, queue_size=1)
        self.waypointPub = rospy.Publisher("/gem/waypoint", ModelState, queue_size=1)


        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)



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


    def rearWheelModel(self, ackermannCmd):
        """
            Description:
                Contains the mathematical model that will represent the vehicle in gazebo

            Inputs:
                ackermannCmd (AckermannDrive): contains desired vehicle velocity and steering angle velocity
                                               that the model should follow

            Returns:
                A List containing the vehicle's x velocity, y velocity, and steering angle velocity
        """
        currentModelState = self.getModelState()

        if not currentModelState.success:
            return

        ## TODO: Compute Bicyle Model

        attitude = currentModelState.pose.orientation
        attitude = self.quaternion_to_euler(attitude.x, attitude.y, attitude.z, attitude.w)
        theta = attitude[2]

        x_dot = ackermannCmd.speed * math.cos(theta)
        y_dot = ackermannCmd.speed * math.sin(theta)
        theta_dot = ackermannCmd.steering_angle_velocity

        return [x_dot, y_dot, theta_dot]    ## TODO: Return x velocity, y velocity, and steering angle velocity

    def rearWheelFeedback(self, currentState, targetState):
        """
            Description:
                Feedback loop which drives the vehicles to the current waypoint

            Inputs:
                currentState (ModelState): The curret state of the vehicle in gazebo
                targetState  (ModelState): The desired target state of the vehicle in gazebo

            Returns:
                ackermannCmd (AckermannDrive): Will be used to compute the new x,y, and steering angle
                                               velocities of the model
        """

        targetVel = math.sqrt((targetState.twist.linear.x*targetState.twist.linear.x) + ((targetState.twist.linear.y*targetState.twist.linear.y)))
        targetAngVel = targetState.twist.angular.z

        ## TODO: Compute Error to current waypoint

        x_current = currentState.pose.position.x
        y_current = currentState.pose.position.y

        attitude_current = currentState.pose.orientation
        attitude_current = self.quaternion_to_euler(attitude_current.x, attitude_current.y, attitude_current.z, attitude_current.w)
        theta_current = attitude_current[2]

        x_target = targetState.pose.position.x
        y_target = targetState.pose.position.y

        attitude_target = targetState.pose.orientation
        attitude_target = self.quaternion_to_euler(attitude_target.x, attitude_target.y, attitude_target.z, attitude_target.w)
        theta_target = attitude_target[2]

        rotation_matrix = np.array([
		        [ math.cos(theta_current), math.sin(theta_current), 0],
		        [-math.sin(theta_current), math.cos(theta_current), 0],
		        [						0,						 0, 1],
        ])

        error_vector = np.array([
		        [x_target 	  - x_current	 ],
		        [y_target 	  - y_current	 ],
		        [theta_target - theta_current]
        ])

        error_vector = np.dot(rotation_matrix, error_vector)
        error_vector = np.transpose(error_vector)[0]

        ## TODO: Create new AckermannDrive message to return

        k_1 = 0
        k_2 = 0
        k_3 = 0
        gain_set = 3

        if gain_set == 1:
            k_1 = 1
            k_2 = 1
            k_3 = 1
        elif gain_set == 2:
            k_1 = 1
            k_2 = 2
            k_3 = 1.25
        elif gain_set == 3:
            k_1 = 2.1
            k_2 = 3.0
            k_3 = 1.5
        elif gain_set == 4:
            k_1 = 10
            k_2 = 20
            k_3 = 10

        ackermannCmd = AckermannDrive()
        ackermannCmd.speed = targetVel * math.cos(error_vector[2]) + k_1 * error_vector[0]
        ackermannCmd.steering_angle_velocity = targetAngVel + targetVel * (k_2 * error_vector[1] + k_3 * math.sin(error_vector[2]))

        return ackermannCmd

    def setModelState(self, currState, targetState):
        """
            Description:2.563
                Sets state of the vehicle in gazebo.

                This function is called by mp2.py at a frequency of apporximately 100Hz

            Inputs:
                currState   (ModelState): The curret state of the vehicle in gazebo
                targetState (ModelState): The desired target state of the vehicle in gazebo

            Returns:
                None
        """

        ## TODO: call controller and model functions
        ackermannCmd = self.rearWheelFeedback(currState, targetState)
        state = self.rearWheelModel(ackermannCmd)

        newState = ModelState()
        newState.model_name = 'polaris'
        newState.pose = currState.pose
        newState.twist.linear.x = state[0]		# TODO: Add x velocity
        newState.twist.linear.y = state[1]     # TODO: Add y velocity
        newState.twist.angular.z = state[2]     # TODO: Add steering angle velocity
        self.modelStatePub.publish(newState)

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
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [roll, pitch, yaw]


    def __waypointHandler(self, data):
        """
            Description:
                Callback handler for the /gem/waypoint topic. If a waypoint is published to
                this topic, this function will be called and append the published waypoint to
                the waypoint list.

            Inputs:
                data (ModelState): the desired state of the model

            Returns:
                None
        """
        self.waypointList.append(data)
