tf2odom
=================================
As it name says this node is in charge of getting a transform from the /base_link frame to /odom and converting it to an odometry message.
This message is needed for node asociated with the slalom and in general it is a good practice to have an odometry topic.
Additionally you will find the implementation of a node called imu_node. This code was written for debugging and simulation purposes.

## How to start?
```sh
rosrun tf2odom hello_node
rosrun tf2odom imu_node
```

## Nodes
+ hello_node
+ imu_node 

## Contributor
+ Fernando Espindola
