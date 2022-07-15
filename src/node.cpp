#include "ros/ros.h"
#include <ros/console.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>

#include <custom_msgs/goalUpdate.h>
#include <custom_msgs/goalDetail.h>
#include <custom_msgs/areaReset.h>
#include <custom_msgs/mapUpdate.h>
#include <custom_msgs/pointData.h>
#include <custom_msgs/position.h>

#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"

#include <pcl_ros/transforms.h>

#include <Eigen/Geometry>
#include <sstream>

ros::Publisher maps_pub;
ros::Publisher goal_pub;

ros::Subscriber maps_sub;
ros::Subscriber goal_sub;
ros::Subscriber posi_sub;

ros::ServiceClient exploreAreaClient;
ros::ServiceClient plannerAreaClient;
ros::ServiceClient plannerGoalClient;
ros::ServiceClient controlAreaClient;
ros::ServiceServer serviceGoalRemove;

int seq = 1;
std::string name;
std::string odometry_frame_id;

double x_positive, new_x_positive;
double x_negative, new_x_negative;
double y_positive, new_y_positive;
double y_negative, new_y_negative;
double z_positive, new_z_positive;

double tfDelay;
double updateConstant;
double x, goalX, initialX, initialRol;
double y, goalY, initialY, initialPit;
double z, goalZ, initialZ, initialYaw;

tf2_ros::Buffer* buffer = nullptr;
tf2_ros::TransformListener* listener = nullptr;

void positionCallback(const custom_msgs::position::ConstPtr &msg){
	x = msg->x;
    y = msg->y;
    z = msg->z;
}

void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& msgCloud)
{
    sensor_msgs::PointCloud2 newCloud;
    custom_msgs::mapUpdate mapMsg;
    custom_msgs::pointData position, initialPosition, initialRotation;

	geometry_msgs::TransformStamped transformCloud;
	bool tf_set = false;

	while (!tf_set)
	{
		try{
			transformCloud = buffer->lookupTransform(odometry_frame_id, msgCloud->header.frame_id, ros::Time::now());
			tf_set = true;
		} catch (tf2::TransformException ex){
			//ROS_ERROR("%s", ex.what());
			tf_set = false;
			ros::Duration(tfDelay).sleep();
		}
	}

    tf2::doTransform(*msgCloud, newCloud, transformCloud);

    position.x = x;
    position.y = y;
    position.z = z;

    initialPosition.x = initialX;
    initialPosition.y = initialY;
    initialPosition.z = initialZ;

    initialRotation.x = initialRol;
    initialRotation.y = initialPit;
    initialRotation.z = initialYaw;

    mapMsg.header.seq = seq;
    mapMsg.header.stamp = ros::Time::now();
    mapMsg.robotName = name;
    mapMsg.pointcloud = newCloud;
    mapMsg.position = position;
    mapMsg.initialPosition = initialPosition;
    mapMsg.initialRotation = initialRotation;

    maps_pub.publish(mapMsg);
}

void goalCallback(const custom_msgs::goalUpdate &msg){

	for (int i=0; i< msg.robotNames.size(); i++)
    {
		if (name == msg.robotNames[i])
		{
            custom_msgs::areaReset newArea;

            goalX = msg.newGoals[i].x;
            goalY = msg.newGoals[i].y;
            goalZ = msg.newGoals[i].z;

            newArea.request.x = msg.newGoals[i].x;
            newArea.request.y = msg.newGoals[i].y;
            newArea.request.z = msg.newGoals[i].z;
            newArea.request.completion = msg.completion[i];

            if ((goalX>x_positive)&&(goalX>0))
            {
                new_x_positive = goalX*updateConstant;
                new_x_negative = x_negative;
            } 
            else if ((goalX<(-1*x_negative))&&(goalX<0))
            {
                new_x_negative = std::abs(goalX)*updateConstant;
                new_x_positive = x_positive;
            } 
            else 
            {
                new_x_negative = x_negative;
                new_x_positive = x_positive;
            }

            if ((goalY>y_positive)&&(goalY>0))
            {
                new_y_positive = goalY*updateConstant;
                new_y_negative = y_negative;
            } 
            else if ((goalY<(-1*y_negative))&&(goalY<0))
            {
                new_y_negative = std::abs(goalY)*updateConstant;
                new_y_positive = y_positive;
            } 
            else 
            {
                new_y_negative = y_negative;
                new_y_positive = y_positive;
            }

            if ((goalZ>z_positive)&&(goalZ>0))
            {
                new_z_positive = goalZ*updateConstant;
            }
            else 
            {
                new_z_positive = z_positive;
            }

            newArea.request.x_positive = new_x_positive;
            newArea.request.x_negative = new_x_negative;
            newArea.request.y_positive = new_y_positive;
            newArea.request.y_negative = new_y_negative;
            newArea.request.z_positive = new_z_positive;

            ROS_DEBUG_STREAM("client_node : new x positive = " << new_x_positive);
            ROS_DEBUG_STREAM("client_node : new x negative = " << new_x_negative);
            ROS_DEBUG_STREAM("client_node : new y positive = " << new_y_positive);
            ROS_DEBUG_STREAM("client_node : new y negative = " << new_y_negative);
            ROS_DEBUG_STREAM("client_node : new z positive = " << new_z_positive);

            if (exploreAreaClient.call(newArea)){
                ROS_DEBUG("client_node : explore node updated");
            } else {
                ROS_ERROR("client_node : failed to call exploreAreaClient");
            }

            if (controlAreaClient.call(newArea)){
                ROS_DEBUG("client_node : control node updated");
            } else {
                ROS_ERROR("client_node : failed to call controlAreaClient");
            }

            if (plannerAreaClient.call(newArea)){
                ROS_DEBUG("client_node : planner node updated");
            } else {
                ROS_ERROR("client_node : failed to call plannerAreaClient");
            }
        }
    }
}

bool removeCallback(custom_msgs::goalDetail::Request &request, custom_msgs::goalDetail::Response &response)
{
    custom_msgs::goalUpdate msg;
    custom_msgs::pointData goal;

    goal.x = request.x;
    goal.y = request.y;
    goal.z = request.z;

    msg.newGoals.push_back(goal);
    msg.robotNames.push_back(name);

    goal_pub.publish(msg);

	response.success = true;

	return true;
}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "client_node");
    ros::NodeHandle node;

    ROS_INFO("Initialized the client_node");

    node.param<std::string>("robot/name",       name,                   "");
    node.param("robot/tfUpdateDelay",           tfDelay,                0.01);

	node.param<std::string>("map/frame",        odometry_frame_id,      "odom");

    node.param("area/front",                    x_positive,             5.0);
    node.param("area/back",                     x_negative,             0.0);
    node.param("area/left",                     y_positive,             5.0);
    node.param("area/right",                    y_negative,             0.0);
    node.param("area/height",                   z_positive,             0.75);
    node.param("area/updateConstant",           updateConstant,         1.1);
    node.param("robot/initial/X",               initialX,               0.00);
    node.param("robot/initial/Y",               initialY,               0.00);
    node.param("robot/initial/Z",               initialZ,               0.00);
    node.param("robot/initial/Roll",            initialRol,             0.00);
    node.param("robot/initial/Pitch",           initialPit,             0.00);
    node.param("robot/initial/Yaw",             initialYaw,             0.00);

    buffer = new tf2_ros::Buffer();
    listener = new tf2_ros::TransformListener(*buffer);

    ROS_INFO("client_node : loaded parameters");

    //  -------------------------------- publisher for sending data to server ------------------------
    maps_pub = node.advertise<custom_msgs::mapUpdate>("/MapReceive", 1, true);
    goal_pub = node.advertise<custom_msgs::goalUpdate>("/RejectedGoals", 1, true);
    
    ROS_INFO("client_node : created publishers");

    //  ------------------------------ subscribers for data coming from server ---------------------------------
    goal_sub = node.subscribe("/GoalBroadcast", 1, goalCallback);

    //  ------------------------------ subscribers for data to be sent to server---------------------------------
    maps_sub = node.subscribe("pointCloud", 1, mapCallback);
    posi_sub = node.subscribe("position", 1, positionCallback);

    ROS_INFO("client_node : created subscribers");

    //  -------------------------- Server clients for reseeting values -------------------------------
    exploreAreaClient = node.serviceClient<custom_msgs::areaResetRequest, custom_msgs::areaResetResponse>("areaResetExplore");
    controlAreaClient = node.serviceClient<custom_msgs::areaResetRequest, custom_msgs::areaResetResponse>("areaResetControl");
    plannerAreaClient = node.serviceClient<custom_msgs::areaResetRequest, custom_msgs::areaResetResponse>("areaResetPlanner");

    serviceGoalRemove = node.advertiseService<custom_msgs::goalDetailRequest, custom_msgs::goalDetailResponse>("goalRemoveServer", removeCallback);

    ROS_INFO("client_node : created services");

    ros::AsyncSpinner spinner(3);
    spinner.start();

    ros::waitForShutdown();
    
    return 0;
}