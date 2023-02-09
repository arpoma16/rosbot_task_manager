#!/usr/bin/env python3
import threading
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState


from rosbot_task_manager.srv import Command_service,Command_serviceResponse
from run_command import Command_move
from get_yalm import get_command

Status_robot = 0 #[0: libre], [1:ocupado]
Current_command = 0 
list_command = []
Current_command_description=[]

runCommand = None

def process_command(req):
    global Status_robot,Current_command,list_command,Current_command_description
    if req.a == "cancelar":
        print("cancel command")
        return False
    # solo si robot no esta ocupado
    if Status_robot == 0:
        for c in range(0,list_command['command_num']) :
            #print(list_command['command']['command'+str(c)])
            command = list_command['command']['command'+str(c)]
            #print("imprimiendo command")
            #print(command)
            if command['name'] == req.a:
                Current_command_description = command
                print("imprimiendo command")
                print(Current_command_description)
                Status_robot = 1
                print("--cambio estatus:"  +  str(Status_robot) )
                
    return Command_serviceResponse(Status_robot)

def getStatusRobot():
    global Status_robot
    return Status_robot

def setStatusRobot(new_status):
    global Status_robot
    Status_robot =new_status
    return Status_robot

def ExecuteCommand():
    global  Current_command_description,runCommand
    print('Ejecutar comando')  

    while True :
        Status_robot = getStatusRobot()
        if Status_robot==1:
            print(Current_command_description['name'])
            if Current_command_description['name']=="bailar":
                print("commando bailar")
                runCommand.dance()
                setStatusRobot(0)
            elif Current_command_description['name']=="vuelta":
                print("vuelta")
                #runCommand.make_spin()
                setStatusRobot(0)
            else:
                print("move to goal")
                print(Current_command_description['name'])
                print(Current_command_description['x'])
                print(Current_command_description['y'])
                print(Current_command_description['angle'])

                runCommand.move_to_goal(float(Current_command_description['x']),float(Current_command_description['y']),float(Current_command_description['angle']))
                runCommand.dance()
                #runCommand.make_spin(3.14,0.0)
                #runCommand.make_spin(3.14,0.0)
                runCommand.move_to_goal(0.0,0.0,0.0)
                setStatusRobot(0)
        rospy.sleep(0.5)

def publish_status(msg):
    global Status_robot
    ## suscribirse a 1 topic para publicar y no estar haciendo otro hilo
    pub = rospy.Publisher('task_manager', String, queue_size=10)
    pub.publish("status:"+str(Status_robot))


def cancel_command():
    # en el caso de cancelar move_base rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {} https://answers.ros.org/question/57772/how-can-i-cancel-a-navigation-command/
    # ver la opcion de matar el hilo de execute command
    # o porner un  breack en cada paso
    global Current_command,Current_command_description
    if Current_command_description["name"]!="dance" or Current_command_description["name"]!="dance" :
        rospy.loginfo("Cancel command")
    rospy.loginfo("delete thread")
    rospy.loginfo("up thread again")

def main():
    rospy.Service('Service_command', Command_service, process_command)
    rospy.Subscriber("battery",BatteryState, publish_status)

    rospy.loginfo("Ready to recive command")
    rospy.spin()


if __name__ == '__main__':
    runCommand = Command_move()
    rospy.init_node('Service_command_server')
    path_yalm = rospy.get_param('~yalm_path')
    list_command = get_command(path_yalm)
    x = threading.Thread(target=ExecuteCommand)
    x.start()
    main()
    
