
#!/usr/bin/env python3
import rospy
import tf

#import tf2_ros
from math import radians, pi
import numpy

from geometry_msgs.msg import Twist, Vector3 ,Transform
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from geometry_msgs.msg import Pose,Point,Quaternion, Twist


class Command_move():

    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        pass

    def dance(self):
        twist = Twist()
        twist.linear = Vector3(0,0,0)
        for i in range(1,4):
            twist.angular = Vector3(0,0,-1)
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(1)
            twist.angular = Vector3(0,0,1)
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(1)
        twist.angular = Vector3(0,0,0)
        self.cmd_vel_pub.publish(twist)

    def move_to_goal(self,goal_x,goal_y,goal_theta):
        # revisar este codigo
        #https://github.com/pirobot/ros-by-example/blob/master/rbx_vol_1/rbx1_nav/nodes/move_base_square.py

        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.move_base = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # Waits until the action server has started up and started listening for goals.
        self.move_base.wait_for_server(rospy.Duration(60))

        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # move to position and rotation 
        quaternion = tf.transformations.quaternion_from_euler(0.0,0.0,goal_theta, axes='sxyz')
        goal.target_pose.pose = Pose(Point(goal_x,goal_y,0.0),Quaternion(*quaternion))
        
        
        # Sends the goal to the action server.
        self.move_base.send_goal(goal)
        # Waits for the server to finish performing the action.
        wait = self.move_base.wait_for_result(rospy.Duration(60))
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            self.move_base.cancel_goal()
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            #return move_base.get_result()
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
            return state

    def shutdown(self):
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def cancel_command(self):
        # en el caso de cancelar move_base rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {} https://answers.ros.org/question/57772/how-can-i-cancel-a-navigation-command/
        print("Cancel command")
        self.move_base.cancel_goal()


    def make_spin(self,radians=3.14,clockwise = True):
        while radians < 0.0 :
            radians += 2*pi
        while radians > 2*pi:
            radians -=2 * pi

        base_cmd = Twist()
        listener = tf.TransformListener()
        now = rospy.Time.now()
        
        listener.waitForTransform("/base_link", "/odom", rospy.Time.now(), rospy.Duration(1.0))

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

            self.cmd_vel_pub.publish(base_cmd)
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
                angle_turned = 2 * pi -angle_turned

            if angle_turned>radians:
                done = True
        if done: 
            return True
        else:
            return False

if __name__ == '__main__':
    
    rospy.init_node('run_commander_node')
    rospy.loginfo("Nodo run commander")
    try:
        uno = Command_move()
        uno.make_spin()

    except rospy.ROSInitException:
        rospy.loginfo("Navigation test finished.")

    
    rospy.loginfo("end node")
