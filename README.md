# rosbot_task_manager

## About this project 

This project is develeped like a project of Robotic Project of MIERA in Universidad de Sevilla.

This project uses a  [Rosbot 2R](https://husarion.com/tutorials/howtostart/rosbot2r-quick-start/) it goint to receive commands (it could by voice, QR, telegram message), in order to execute tasks such as dropping something off at a point where it will have to retun to home when for finish the task.

This project is based on [rosbot_patrol](https://github.com/adamkrawczyk/rosbot_patrol) ,[robot patrol simulation](https://github.com/adamkrawczyk/rosbot_patrol_simulation) that have this [tuturial in husarion web page](https://husarion.com/tutorials/ros-projects/security-guard-robot/)


##  Considerations!! this is really important
*  no upgrade the RosBot 2R when you upgrade  the astra driver and wifi driver fail.
*  dont try to use the sound pub of raspberry pi 4 that it have, this have some confilcts with the camera after you dont have video when up the astra container.

* if you want get commands for audio  i dont test but you can use a usb sound adapter  or other device that send it for ros.

## how run 

### Docker services for rosbot 2R
Encender los servicios( esto se tiene que hacer usando vnc)

    docker compose up -d rosbot ros-master          (roscore and ekf)
    docker compose up -d rosbot rosbot              (ekf)
    docker compose up -d rosbot frame-transformer
    docker compose up -d rosbot rplidar             (lidar)
    docker compose up -d rosbot astra               (camara)

apagar los servicios

    docker compose down

Para poder ejecutar docker mediante SSH  tienes que exportar

    export DISPLAY=:1

### Setup ros master 


se cambio la ip  de __ROS_MASTER_IP__ and __ROS_IP en__ la computadora cliente y en el robot.
descargar ros_description  y anadir al source del bash    o ejecutar en consola.

    export ROS_IP=192.168.175.177
    export ROS_MASTER_URI=http://192.168.175.120:11311
    source /opt/ros/noetic/setup.bash


### Revisar servicios

    rossrv show speech_recognition/Command_service.srv

    rosrun speech_recognition TaskManager.py




## notas
para poder realizar rostopic echo  hay que levantar el puerto 1234 y el 2345

puedes revisar este [TUTORIAL](https://medium.com/@yasuhirachiba/specifying-port-to-be-used-by-ros1-node-bd9dfd8643c6)


### Error al instalar pyaoudio
https://bobbyhadz.com/blog/python-error-could-not-build-wheels-for-pyaudio


    rosrun teleop_twist_keyboard teleop_twist_keyboard.py


## Referecias
1. [Rosbot patrol tutorial](https://husarion.com/tutorials/ros-projects/security-guard-robot/)

1. [Speech reconition](https://realpython.com/python-speech-recognition/)



## Authors

* [Alvaro Poma](https://github.com/arpoma16)
* Nelson Molina  