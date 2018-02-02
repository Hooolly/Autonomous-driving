#ifndef POINT_CATCHER_H
#define POINT_CATCHER_H

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

//#include <igl/cotmatrix.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
//#include <Eigen/Dense>
#include <Eigen/Geometry>

//#include <Eigen3/Core>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

//#include <Eigen/Dense>

// Library for geometry processing
//#include <igl/procrustes.h>
#include <iostream>

using namespace std;

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;
static const double A = 1.80; //1.0
static const double B = 4.00; //2.5  

class PointCatcher 
{
public:
    // Tunable parameters
    constexpr static double TOLERANCE_DIAMETER = 0.15;
    constexpr static double TOLERANCE_DIAMETER_FINAL = 0.25;
    constexpr static double SECURITY_RADIO = 0.60;
    constexpr static int MINIMUM_COUNT_TO_FILTER = 100;
    constexpr static int LOWER_COUNT_THRESHOLD = 10;
    constexpr static int STOP_TRANSFORMING = 30;
    constexpr static double LOG_MULTIPLIER = 0.00;


    // Constructor
    PointCatcher();

private:
    //declare Node
    ros::NodeHandle node;

    //declare publishers
    ros::Publisher point_pub;
    ros::Publisher angelina_pub;
    ros::Publisher infield_pub;

    //declare subscribers
    ros::Subscriber subs_green;
    ros::Subscriber subs_blue;
    ros::Subscriber subs_yellow;

    ros::Subscriber robot_pose;

    //vector with the points for the found objects
    std::vector<Eigen::Vector3d> objects;
    std::vector<Eigen::Vector3d> objects_f; //objects filtered
    Eigen::VectorXd corner0;


    long objectsCount;
    long transformCount;

    float A_detected;
    float B_detected;
    float ab_ratio;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    tf::TransformBroadcaster broadcasterMap;
    tf::StampedTransform tfMap2Odom;


    bool areThere2Corners;
    bool transformIsReady;

    Eigen::Vector2d odomAtStart;
    bool odomStarted;

    //Poles coordinates
    Eigen::MatrixXd X_6; // Based on 6 poles
    Eigen::MatrixXd X_8; // Based on 8 poles
    Eigen::MatrixXd X_full; // Based on 8 poles

    // Transformation data after Procrustes analysis
    double scale;
    Eigen::MatrixXd R; //Rotation
    Eigen::VectorXd t; //Translation

    //store a set of points everytime a message comes
    void savePoint(const geometry_msgs::PointStamped pylon_msg);

    //Check if point is too close to values already stored
    bool addNewPoint(geometry_msgs::Point newPoint);

    //Select the values that appear more often
    bool filterHistogram();

    void calculateProcrustes();

    void localization(nav_msgs::Odometry msg);

    void sendPolesTransforms();

    void calculatePolesCoordinates();

    //Sort array for correctness of procrustes analysis

    void printVectorAsMatrix(std::vector<Eigen::Vector3d> vec, std::string name);

    bool hasValuesBetween(Eigen::VectorXd vect, float lowLimit, float highLimit);

    double findMinimum(Eigen::MatrixXd Ydif);


};

#endif // POINT_CATCHER_H


