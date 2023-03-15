import math
import numpy as np
import rospy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive


class bicycleModel():

    def __init__(self):

        self.length = 1.88
        pt1 = ModelState()
        pt2 = ModelState()
        pt3 = ModelState()

        pt1.pose.position.x = -10
        pt1.pose.position.y = -10
        pt1.twist.linear.x = .25
        pt1.twist.linear.y = .25
        #pt1.twist.angular = 0

        pt2.pose.position.x = 10
        pt2.pose.position.y = 10
        pt2.twist.linear.x = .25
        pt2.twist.linear.y = .25
        #pt2.twist.angular = 0

        pt3.pose.position.x = 0
        pt3.pose.position.y = 0
        pt1.twist.linear.x = .25
        pt1.twist.linear.y = .25
        #pt3.twist.angular = 0

        self.waypointList = []

        self.waypointSub = rospy.Subscriber("/gem/waypoint", ModelState, self.__waypointHandler, queue_size=1)
        self.waypointPub = rospy.Publisher("/gem/waypoint", ModelState, queue_size=1)

        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)

    def getModelState(self):
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

        k_1 = 4
        k_2 = 15
        k_3 = 4

        ackermannCmd = AckermannDrive()
        ackermannCmd.speed = targetVel * math.cos(error_vector[2]) + k_1 * error_vector[0]
        ackermannCmd.steering_angle_velocity = targetAngVel + targetVel * (k_2 * error_vector[1] + k_3 * math.sin(error_vector[2]))

        return ackermannCmd

    def setModelState(self, currState, targetState):

        #control = AckermannDrive()
        control = self.rearWheelFeedback(currState, targetState)
        values = self.rearWheelModel(control)

        newState = ModelState()
        newState.model_name = 'polaris'
        newState.pose = currState.pose
        newState.twist.linear.x = values[0]
        newState.twist.linear.y = values[1]
        newState.twist.angular.z = values[2]
        self.modelStatePub.publish(newState)

    def quaternion_to_euler(self, x, y, z, w):
        x, y, z, w = float(x), float(y), float(z), float(w)

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

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def __waypointHandler(self, data):
        self.waypointList.append(data)

    #add a list of points in ModelState
    def addPlanedPath(self, path):
        self.waypointList = path + self.waypointList

    def popNextPoint(self):
        if self.waypointList:
            return self.waypointList.pop(0)
        else:
            return None
