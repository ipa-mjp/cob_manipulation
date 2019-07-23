#include <iostream>
#include <string.h>
#include <map>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <cob_control_msgs/ObstacleDistances.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <moveit_msgs/PlanningScene.h>
#include <sensor_msgs/PointCloud2.h>

class DebugObstacleDistance
{

ros::NodeHandle nh_;
ros::Publisher marker_pub_;
ros::Subscriber obstacle_distances_sub_, pointCloud_obstacle_distances_sub_;

public:
   int init()
    {
        marker_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("distance_markers", 1, true);
        obstacle_distances_sub_ = this->nh_.subscribe("/monitored_planning_scene", 1, &DebugObstacleDistance::obstacleDistancesCallback, this);
        pointCloud_obstacle_distances_sub_ = this->nh_.subscribe("/obstacle_distance_moveit/cam3d_left/filtered_cloud", 1, &DebugObstacleDistance::pointCloudObstacleDistancesCallback, this);


        ros::Duration(1.0).sleep();
        ROS_WARN("Debug initialized.");
        return 0;
    }

     void obstacleDistancesCallback(const moveit_msgs::PlanningScene::ConstPtr& msg)
    {
        ;
        //ROS_INFO_STREAM(msg->name);
    } 

    void pointCloudObstacleDistancesCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        visualization_msgs::MarkerArray marker_array;

        ROS_INFO_STREAM(msg->header);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

        ROS_INFO_STREAM("Size: "<< cloud->points.size ()); 
        ROS_INFO_STREAM("width: "<< cloud->width); 
        ROS_INFO_STREAM("height: "<< cloud->height);    
        ROS_INFO("==============");
        

        
        for (size_t i = 0; i < cloud->points.size (); i=i+10)
        {

            visualization_msgs::Marker marker_vector;

            marker_vector.type = visualization_msgs::Marker::ARROW;
            marker_vector.lifetime = ros::Duration();
            marker_vector.action = visualization_msgs::Marker::ADD;
            marker_vector.ns = "arrows";
            marker_vector.header = msg->header;
            
            marker_vector.scale.x = 0.01;
            marker_vector.scale.y = 0.05;
            
            marker_vector.color.a = 1.0;
            marker_vector.color.b = 1.0;
            
            geometry_msgs::Point start;
            start.x = 0;
            start.y = 0;
            start.z = 0;

            
            marker_vector.id = 2*i;
            //ROS_INFO_STREAM(cloud->points[i].x);
            geometry_msgs::Point end;
            end.x = cloud->points[i].x;
            end.y = cloud->points[i].y;
            end.z = cloud->points[i].z;
            
            marker_vector.points.push_back(start);
            marker_vector.points.push_back(end);
            marker_array.markers.push_back(marker_vector);
        }
        marker_pub_.publish(marker_array);
        //pcl::PointCloud<pcl::PointXYZ> cloud;
        //pcl::fromROSMsg(*msg, cloud);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "debug_obstacle_distance_node");

    DebugObstacleDistance dod;
    if (dod.init() != 0)
    {
        ROS_ERROR("Failed to initialize DebugDistanceManager.");
        return -1;
    }

    ros::spin();
}