import math
import time
import matplotlib.pyplot as plt

import rospy
from gazebo_msgs.msg import  ModelState
from controller import bicycleModel

def plot_trajectory(position_x, position_y, waypoints, save=False):
    handles = []
    legends = []

    fig = plt.figure()
    plt.title("Vehicle Trajectory")
    plt.xlabel("Position X")
    plt.ylabel("Position Y")

    handle, = plt.plot(position_x, position_y)
    handles.append(handle)
    legends.append("Trajectory")

    for i in range(0, len(waypoints)):
        handle, = plt.plot(waypoints[i].pose.position.x, waypoints[i].pose.position.y, "o")
        handles.append(handle)
        legends.append("Waypoint " + str(i + 1))

    plt.legend(handles, legends)
    plt.show()

    if save == True:
        fig.savefig("trajectory.png")

    return

if __name__ == "__main__":
    rospy.init_node("model_dynamics")

    model = bicycleModel()

    waypoints = list(model.waypointList)

    endList = 0;

    #wait till a waypoint is received
    while(not model.waypointList):
        pass
        rospy.sleep(7)

    targetState = ModelState()
    targetState = model.waypointList.pop(0)

    position_x = []
    position_y = []

    start = time.time()

    rate = rospy.Rate(100)  # 100 Hz
    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state

        currState =  model.getModelState()
        if not currState.success:
            continue


        distToTargetX = abs(targetState.pose.position.x - currState.pose.position.x)
        distToTargetY = abs(targetState.pose.position.y - currState.pose.position.y)

        position_x.append(currState.pose.position.x)
        position_y.append(currState.pose.position.y)

        if(distToTargetX < 1 and distToTargetY < 1):
            if not model.waypointList:
                newState = ModelState()
                newState.model_name = 'polaris'
                newState.pose = currState.pose
                newState.twist.linear.x = 0
                newState.twist.linear.y = 0
                newState.twist.angular.z = 0
                model.modelStatePub.publish(newState)
                #only print time the first time waypontList is empty
                if(not endList):
                    endList = 1
                    end = time.time()
                    print("Time taken:", end-start)
                    plot_trajectory(position_x, position_y, waypoints, True)
            else:
                if(endList):
                    start = time.time()
                    endList = 0
                targetState = model.waypointList.pop(0)
                markerState = ModelState()
                markerState.model_name = 'marker'
                markerState.pose = targetState.pose
                model.modelStatePub.publish(markerState)
        else:
            model.setModelState(currState, targetState)
            print(distToTargetX,distToTargetY)
            markerState = ModelState()
            markerState.model_name = 'marker'
            markerState.pose = targetState.pose
            model.modelStatePub.publish(markerState)




    rospy.spin()
