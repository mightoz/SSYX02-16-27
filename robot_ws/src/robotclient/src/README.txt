EADME
From generic to specific robot set up for services

service_Rotate:
rosrun robotclient serviceRotate.py __name:=rotateRobot_service1 rotateRobot:=rotateRobot1 RosAria/cmd_vel:=robot1/cmd_vel

get_coord_server.py:
rosrun robotclient get_coord_server.py __name:=get_coord_server1 get_coord:=get_coord1 _ip_of_uwb:=101


service_MoveForward.py:
rosrun robotclient serviceMoveForward.py __name:=moveRobot_server1 moveRobot:=moveRobot1 RosAria/cmd_vel:=robot1/cmd_vel

UpdateTwistServer.py:
rosrun robotclient UpdateTwistService.py __name:=update_twist_server_1 updateTwist:=updateTwist1 RosAria/cmd_vel:=robot1/cmd_vel

