slalom
=================================
This package implements a traffic cone detector, a managing node for transforms and the node in charge of the actual slalom.
The key part of the slalom can be found in the constructor called *Slalom()*. In that section goal are iterativetly defined for a set of 6 traffic cones.

## How to start?
```sh
rosrun slalom pylon_detector_node
rosrun slalom map_node
rosrun slalom slalom_node
```
## Nodes
+ pylon_detector_node
+ map_node 
+ slalom_node

## Contributor
+ Fernando Espindola
