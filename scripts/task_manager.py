#!/usr/bin/env python3
import threading
import rospy
from std_msgs.msg import String

import run_command
from rosbot_task_manager.srv import Command_service,Command_serviceResponse

import get_yalm

Status_robot = 0 #[0: libre], [1:ocupado]
Current_command = 0 
list_command = []
Current_command_description=[]

        

def process_command(req):
    global Status_robot,Current_command,list_command,Current_command_description
    if req.a == "cancelar":
        print("cancel command")
        return False
    # solo si robot no esta ocupado
    if Status_robot == 0:
        for command in list_command["command"]:
            if command["name"] == req.a:
                Current_command_description = command
                Status_robot =1
                
    return Command_serviceResponse(Status_robot)


def ExecuteCommand():
    print('Ejecutar comando')  
    global Status_robot, Current_command_description

    while True :
        if Status_robot==1:
            if Current_command_description["name"]=="dance":
                print("dance")
                run_command.dance()
            elif Current_command_description["name"]=="vuelta":
                print("vuelta")
                run_command.make_spin()
            else:
                print("move to goal")
                run_command.move_to_goal(Current_command_description["x"],Current_command_description["y"],Current_command_description["angle"])
                run_command.make_spin(3.14,0)
                run_command.make_spin(3.14,0)
                run_command.move_to_goal(0,0,0)
        Status_robot = 0
        rospy.sleep(0.5)

def publish_status():
    global Status_robot
    ## suscribirse a 1 topic para publicar y no estar haciendo otro hilo
    pub = rospy.Publisher('task_manager', String, queue_size=10)
    pub.publish("status:"+str(Status_robot))


def cancel_command():
    # en el caso de cancelar move_base rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {} https://answers.ros.org/question/57772/how-can-i-cancel-a-navigation-command/
    # ver la opcion de matar el hilo de execute command
    # o porner un  breack en cada paso
    global Current_command
    if Current_command_description["name"]!="dance" or Current_command_description["name"]!="dance" :
        rospy.loginfo("Cancel command")
    rospy.loginfo("delete thread")
    rospy.loginfo("up thread again")

    
    

def main():
    rospy.Service('Service_command', Command_service, process_command)
    rospy.Subscriber("battery", "battery", publish_status)

    rospy.loginfo("Ready to recive command")
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('Service_command_server')
    path_yalm = rospy.get_param("yalm_path")
    list_command= get_yalm.get_command(path_yalm)
    x = threading.Thread(target=main)
    x.start()
    main()
    
