# asl_flight2

This is a ROS2 package. The suggested folder structure is given below.

    .
    |── quadrotors            # ROS2 workspace directory
    |   |── src
    |   |   |── asl_flight2   # this package
    |   |   |── px4_msgs
    |   |   |── px4_ros_com
    |   |── ...
    |── ...


# to run sim
in a new terminal run
```MicroXRCEAgent udp4 -p 8888```
then start the sim in a new terminal using 
```make px4_sitl gz_x500```
