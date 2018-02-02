#include <ros/ros.h>
#include "point_catcher.h"

/*
 * @author Fernando Espindola
 */


/**
 *
 * @brief PointCatcher::PointCatcher The PointCatcher class processes data from the LaserScan and finds
 *        a transformation between the /odom frame and the pylons frames. The name /map was not used to avoid
 *        conflicts with the cost map from the MoveBase node.
 *        The required tfs is then broadcasted to be used by the Global Navigation.
 *
 */
PointCatcher::PointCatcher()
{

    subs_green = node.subscribe("/pylon", 1000, &PointCatcher::savePoint, this);

    robot_pose = node.subscribe("/odom_alt", 1000, &PointCatcher::localization, this);

    //point_pub = node.advertise<geometry_msgs::Point>("map/A_B", 1000);

    //angelina_pub = node.advertise<std_msgs::Float32>("map/ab_ratio", 1000);

    //infield_pub = node.advertise<std_msgs::Bool>("map/in_field", 1000);

    tfMap2Odom.frame_id_ = std::string("odom");
    tfMap2Odom.child_frame_id_ = std::string("slalom");

    areThere2Corners = false;

    odomStarted = false;

    transformIsReady = false;

    objectsCount=0;

    transformCount=0;

    //A_detected = A;
    //B_detected = B;
    //ab_ratio = 0;

    //Coordinates for the closest six poles
    X_6= (Eigen::MatrixXd(6,2)<<
          0      ,        0 ,
          0      ,        A ,
          0      ,      1*A ,
          0      ,      2*A ,
          0      ,      3*A ,
          0      ,      4*A ).finished();

    ROS_INFO("-----------------------------");

}

/**
 * @brief PointCatcher::savePoint Every time a new message from the Image Processing comes, it converts the content of the message to the tf format.
 * Transform the coordinates of all new points between the camera frame and the /odom, which is a fixed frame. Afterwards the points are checked for correctness and filtered.
 * @param poles_msg Contains the green poles found by the image processing node. It contains the name of the reference frame for the current points as well as the time from the point cloud.
 */
void PointCatcher::savePoint(const geometry_msgs::PointStamped pylon_msg)
{


    // Prepare a vector to contain a set of points
    std::vector<geometry_msgs::Point> detected_pylones;

    // Fill the vector with the 9 maximal possible points
    detected_pylones.push_back(pylon_msg.point);

    // Shirnk vector to fit the number of detected poles
    detected_pylones.resize(1);

    try{

        ros::Time now  = ros::Time::now();
        ros::Time past = pylon_msg.header.stamp;

        listener.waitForTransform("/odom", pylon_msg.header.frame_id, now, ros::Duration(1.0));

        listener.lookupTransform("/odom", pylon_msg.header.frame_id, now, transform);

        geometry_msgs::PointStamped pylon_laser;

        pylon_laser.header.frame_id = pylon_msg.header.frame_id;
        pylon_laser.header.stamp = past;

        for (int i = 0; i < detected_pylones.size(); i++){

            pylon_laser.point = detected_pylones[i];

            geometry_msgs::PointStamped pylon_odom;

            listener.transformPoint("/odom", pylon_laser, pylon_odom);

            PointCatcher::addNewPoint(pylon_odom.point);

        }
    }

    catch (tf::TransformException ex) {

        ROS_WARN("Laser to Base transform unavailable %s", ex.what());

    }

    //PointCatcher::printVectorAsMatrix(objects,"X");

    //Select the values that appear more often

    if(!transformIsReady){

        if(PointCatcher::filterHistogram()){
            // Find transformation that maps points detected to the characteristics of the game
            //PointCatcher::calculateProcrustes();
            //ROS_INFO("Calculate Procrustes");
        }

    }
}

/**
 * @brief PointCatcher::addNewPoint
 * @param newPoint
 * @return
 */
bool PointCatcher::addNewPoint(geometry_msgs::Point newPoint){

    tf::Vector3 newPoint_tf(newPoint.x, newPoint.y, 0);

    bool valid = true;

    double closestDistance, newDistance;

    closestDistance = 1.0/0.0; // Set to infinity at the beginning

    int i_c = -1;
    int weight;

    Eigen::Vector3d newPointVector;

    if(newPoint.x!=newPoint.x){

        valid = false;
        return valid;

    }

    for (int i = 0; i < objects.size(); i++){

        tf::Vector3 object_i(objects[i](0), objects[i](1), 0);

                newDistance = newPoint_tf.distance(object_i);

        if (newDistance < closestDistance){

            //Find the closest object
            closestDistance = newDistance;
            i_c = i;
        }

    }

    if(closestDistance < TOLERANCE_DIAMETER ) {
        
        // If the point is close enough to any other entry in the array
        // compute a weighted average and save it in the same point-
        // The z-coordinate is used to store the weight

        weight = objects[i_c](2);
        objects[i_c](0) = (objects[i_c](0) * weight + newPoint.x)/(weight+1);
                objects[i_c](1) = (objects[i_c](1) * weight + newPoint.y)/(weight+1);
                objects[i_c](2) =  weight + 1;
        objectsCount++;
        
    } else {
        
        // Add point with weight 1
        newPointVector << newPoint.x, newPoint.y, 1;
        objects.push_back(newPointVector);
        objectsCount++;
    }

    return valid;
}

/**
 * @brief PointCatcher::filterHistogram Calculate and modify a histogram based on every new point.
 * @return
 */
bool PointCatcher::filterHistogram(){


    //ROS_INFO("Entered filterHistogram");
    bool notReady4Procrustes = false;
    bool ready4Procrustes = true;

    if(objectsCount < MINIMUM_COUNT_TO_FILTER){

        return notReady4Procrustes;

    }

    std::vector<Eigen::Vector3d> newObjects, newObjects_f, newObjects_r;

    // Accept only values with occurrences higher than LOWER_COUNT_THRESHOLD
    for (int i = 0; i < objects.size(); i++){

        if(objects[i](2) > LOWER_COUNT_THRESHOLD + LOG_MULTIPLIER * objectsCount){

            newObjects.push_back(objects[i]);

        }

    }

    //PointCatcher::printVectorAsMatrix(newObjects,"newObjects");

    // Get rid of values that are close enough to be
    // considered the same point

    double newDistance, vector_AB_norm, delta_norm;

    Eigen::Vector2d vector_i, vector_j, vector_AB, delta;

    int weight_i, weight_j;

    newObjects_f = newObjects; // Make a copy

    for (int i = 0; i < newObjects.size(); i++){

        vector_i << newObjects[i](0), newObjects[i](1);
        weight_i = newObjects[i](2);

        for (int j = 0; j < newObjects.size(); j++){

            if(j!=i){

                vector_j << newObjects[j](0), newObjects[j](1);
                newDistance = (vector_i-vector_j).norm();
                weight_j = newObjects[j](2);

                // If the new point is closer than A/8 to an existent one
                // calculate a weighted average

                if( newDistance < A/8 && weight_i < weight_j ){

                    newObjects_f.erase(
                                std::remove(newObjects_f.begin(),
                                            newObjects_f.end(),
                                            newObjects[i]),
                                            newObjects_f.end());
                }

            }

        }

    }

    newObjects_r = newObjects_f;


    for (int i = 0; i < newObjects_f.size(); i++){

        vector_i << newObjects_f[i](0), newObjects_f[i](1);

        vector_AB << A , B;

        delta << vector_i(0) - odomAtStart(0), vector_i(1) - odomAtStart(1);

        delta_norm = delta.norm();

        vector_AB_norm = vector_AB.norm() * SECURITY_RADIO;


        // Remove the poles that are too far from the start position to avoid
        // detection of hazardous data

        if(vector_AB_norm < delta.norm()){

            newObjects_r.erase(
                        std::remove(newObjects_r.begin(), //Pointer begin
                                    newObjects_r.end(), // Pointer end
                                    newObjects_f[i]),   // Object to erase
                        newObjects_r.end());
        }

    }

    //PointCatcher::printVectorAsMatrix(newObjects_f,"newObjects_f");

    PointCatcher::printVectorAsMatrix(newObjects_r,"newObjects_r");
    

    objects_f = newObjects_r;

    return ready4Procrustes;

}

/**
 * @brief PointCatcher::sendPolesTransforms Build a vector of transforms by using the current
 *  available data. Then broadcast every single pole for visualization purposes.
 *
 */
void PointCatcher::sendPolesTransforms(){

    ROS_INFO("sendTransforms");

    std::vector<tf::StampedTransform> transforms;

    //for(int i = 0; i < objects_f.size(); i++){
    
    if(objects_f.size()>0){
    
	    for(int i = 0; i < 6; i++){

		tf::StampedTransform transform;
		transform.setIdentity();
		transform.child_frame_id_ = std::string("pylon_")+std::to_string(i);
		transform.frame_id_ = std::string("odom");
		transform.stamp_ = ros::Time::now();

		if (i==0){

		transform.setOrigin(tf::Vector3(objects_f[i](0), objects_f[i](1), 0));
		                    transform.setRotation(tf::Quaternion(0.0f, 0.0f, 0.0f));

		} else{

		//Just look at the first pylon and project
		transform.setOrigin(tf::Vector3(objects_f[0](0) + 1.5 * i, objects_f[0](1) , 0));
		                    transform.setRotation(tf::Quaternion(0.0f, 0.0f, 0.0f));

		}

		transforms.push_back(transform);

	    }

    }

    broadcasterMap.sendTransform(transforms);

    ROS_INFO("End sendTransforms");

}

/**
 * @brief PointCatcher::localization Broadcasts a transformation between /odom frame and /slalom frame
 * every time it receives a new message from the odometry
 * @param msg Message from the /odom topic
 */
void PointCatcher::localization(const nav_msgs::Odometry msg){

    //ROS_INFO("Localization!!!!");

    if(odomStarted==false){

        odomStarted = true;

        odomAtStart << msg.pose.pose.position.x , msg.pose.pose.position.y;

	ROS_INFO("odomAtStart << msg.pose.pose.position.x , msg.pose.pose.position.y;");

    }
 

    if(areThere2Corners){

	tf::Quaternion q;

        // Calculate the rotation angle from the matrix

        //q.setRPY(0, 0, -atan2(R(1,0),R(0,0)));// map as parent
        q.setRPY(0, 0, -atan2(R(1,0),R(0,0))); // map as child

        // Broadcast -odom to map
        tfMap2Odom.stamp_ = ros::Time::now();

        // Specify actual transformation vectors
        tfMap2Odom.setOrigin(tf::Vector3(t(0), t(1), 0.0f));

        tfMap2Odom.setRotation(q);

        // Broadcast transform
        broadcasterMap.sendTransform(tfMap2Odom);

        transformCount++;

        if(transformCount >= STOP_TRANSFORMING ){

            transformIsReady = true;

        } else {

            //ROS_INFO("Transforming :)");

            PointCatcher::printVectorAsMatrix(objects_f,"objects_f");

            //std::cout << "corner0=[ " << corner0 << std::endl << "];"<< std::endl;

        }

    }

    PointCatcher::sendPolesTransforms();
   
}


/**
 * @brief PointCatcher::printVectorAsMatrix Visualize matrix for debug.
 *        The printed matrix can be easily used as input in the MATLAB console.
 * @param vec   Vector of Vector3 data
 * @param name  Matrix name to be printed
 */
void PointCatcher::printVectorAsMatrix(std::vector<Eigen::Vector3d> vec, std::string name){

    std::cout << name << "=[ " << std::endl;

    for (int i = 0; i < vec.size(); i++){

        std::cout << vec[i](0) << " " << vec[i](1) << " " << vec[i](2) << std::endl;

    }

    std::cout << "];" << std::endl;

}


/**
 * @brief PointCatcher::hasValuesBetween Simple algorith to find if the values of a vector are between two limits
 * @param vect  Differences vector
 * @param lowLimit
 * @param highLimit
 * @return  Return true if value is in the interval, and false otherwise
 */
bool PointCatcher::hasValuesBetween(Eigen::VectorXd vect, float lowLimit, float highLimit){

    int x_size = vect.rows();
    double current;

    for(int i = 0; i < x_size; i++){

        current = vect(i);

        if(current > lowLimit && current < highLimit){

            return true;

        }

    }

    return false;

}

/**
 * @brief PointCatcher::findMinimum
 * @param Ydif
 * @return
 */
double PointCatcher::findMinimum(Eigen::MatrixXd Ydif){

    int y_size = Ydif.rows();

    double minimum;

    Eigen::VectorXd minimaVector (y_size);

    minimaVector = Ydif.col(1);

    minimum = (minimaVector(0) + minimaVector(2))/2;

    return minimum;

}



/**
 * @brief PointCatcher::calculatePolesCoordinates Once the new values for A and B are available recalculate the X_full matrix
 */
void PointCatcher::calculatePolesCoordinates(){

    X_full= (Eigen::MatrixXd(14,2)<<
             0  		,      0	    ,
             0      		,    0.3*A_detected ,
             0      		,      1*A_detected ,
             0      		,   1.50*A_detected ,
             0      		,      2*A_detected ,
             0      		,   2.70*A_detected ,
             0      		,      3*A_detected ,
             B_detected      ,      0	    ,
             B_detected      ,   0.3*A_detected  ,
             B_detected      ,      1*A_detected ,
             B_detected      ,   1.50*A_detected ,
             B_detected      ,      2*A_detected ,
             B_detected      ,   2.70*A_detected ,
             B_detected      ,      3*A_detected
             ).finished();

}


