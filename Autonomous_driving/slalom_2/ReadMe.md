Technical Autonomous Systems WS2017/18 G3 

Author：Hao Xu ( hao.xu@tum.de )         


# Method introduction:
This ROS package contains one node Slalom_2 to implement the slalom task of the TAS project. 
The position of each cone is detected once at the begining. 
Trigonometric functions are used to set a limited view angle for the car. 
In this view window, vettel can detect the position of each cone by setting different distance range. 
After these, the waypoints position is calculated and published to the move_base node.
This method is firstly implemented in our simulation environment and then accomplished on the real car.
Under the trailer system theory, this method also works for slalom driving with trailer.


## Brief overview for method 2:
+ Detect all cones according to distance range and view angle
+ Calculate and publish waypoints 
+ Using trailer system to accomplish slalom with trailer


## Files introduction:
+ "nodes": includes main function.
+ "src": includes function library.
+ "include": includes head file.

## Running the software

### Run the slalom_2 node
```
roslaunch slalom_2 slalom.launch

```

## Trailer analysis
The reference essay could be found in "References" file.
The trailer model of Vettel could be found in "final slides".