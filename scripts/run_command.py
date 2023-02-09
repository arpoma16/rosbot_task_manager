
#!/usr/bin/env python3
import rospy
import tf

#import tf2_ros
import numpy

from geometry_msgs.msg import Twist, Vector3 ,Transform

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal

cmd_vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)# name, data tipe and size queue



M_PI= 3.1415

def make_spin(radians=1,clockwise = True):
    while (radians < 0):
        radians += 2*M_PI
    while (radians >2 *M_PI):
        radians -=2 * M_PI

    base_cmd = Twist()
    listener = tf.TransformListener()
    #rate = rospy.Rate(10.0)
    now = rospy.Time.now()
    
      
    listener.waitForTransform("/base_link", "/odom", rospy.Time(), rospy.Duration(1.0))

    start_transform = listener.lookupTransform("/base_link", "/odom", rospy.Time(0))
    base_cmd.linear.x = 0
    base_cmd.linear.y = 0
    base_cmd.angular.z = 0.75
    desired_turn_axis = Vector3( 0, 0, 1)

    if clockwise:
        base_cmd.angular.z = - base_cmd.angular.z
    else:
        desired_turn_axis = -desired_turn_axis

    rate = rospy.Rate(5.0)

    done = False

    while(not done):

        cmd_vel_pub.publish(base_cmd)
        rate.sleep()

        try:
            listener.waitForTransform("/base_footprint", "/odom",rospy.Time(), rospy.Duration(1.0))
            current_transform = listener.lookupTransform("/base_link", "/odom", rospy.Time(0))
        except:
            rospy.loginfo("Error")
            break

        current_transform.transform

        TF_1 = tf.transformations.inverse_matrix(numpy.dot(start_transform)) 
        TF_2 = numpy.dot(current_transform)
        TF_R = numpy.dot(TF_1 ,TF_2)
        TF_R_t = tf.transformations.translation_from_matrix(TF_R)
        TF_R_q = tf.transformations.quaternion_from_matrix(TF_R)
        (roll,pich,yaw) = tf.transformations.euler_from_quaternion(TF_R_q)
        angle_turned = yaw
        if abs(angle_turned)< 0.02:
            done =True

        if numpy.dot( TF_R_t,desired_turn_axis) <0:
            angle_turned = 2 * M_PI -angle_turned

        if angle_turned>radians:
            done = True
    if done: 
        return True
    else:
        return False

def dance():
    twist = Twist()
    twist.linear = (0,0,0)
    twist.angular = (0,0,0)
    for i in range(1,4):
        twist.angular = (0,0,1)
        cmd_vel_pub.publish(twist)
        rospy.sleep(1)
        twist.angular = (0,0,-1)
        cmd_vel_pub.publish(twist)
        rospy.sleep(1)


def move_to_goal(goal_x,goal_y,goal_theta):

    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseGoal)
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # move to position and rotation 
    goal.target_pose.pose.position.x =goal_x
    goal.target_pose.pose.position.y = goal_y
    goal.target_pose.pose.orientation.w = goal_theta
    quaternion = tf.transformations.quaternion_from_euler(0.0,0.0,goal_theta)
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def cancel_command():
    # en el caso de cancelar move_base rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {} https://answers.ros.org/question/57772/how-can-i-cancel-a-navigation-command/
    print("Cancel command")


if __name__ == '__main__':
    
    rospy.init_node('run_commander_node')

    rospy.loginfo("Nodo run commander")
    rospy.loginfo("node has be started")
