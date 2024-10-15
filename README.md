# Agrobot: Robot Control Middleware
Repo for AgroBot project, presenting middleware between Computer Vision and Hyundai controller.
This middleware receives commands from Computer Vision module and controls the Hyundai HH7 
robot arm accordingly. It utilizes OnLTrack functionality available in Hi5a-S20 controller
for real-time robot motion control. Middleware implements the UDP server for communication with
the Hyundai controller and the TCP server for communication with Computer Vision module.
