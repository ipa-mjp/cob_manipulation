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

// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>


class DebugObstacleDistance
{

ros::NodeHandle nh_;
ros::Publisher marker_pub_;
ros::Subscriber obstacle_distances_sub_, pointCloud_obstacle_distances_sub_;
tf::TransformListener transform_listener_;
tf2_ros::StaticTransformBroadcaster static_broadcaster;

public:

   int init()
    {
        //transform_listener_(this->nh_, ros::Duration(300.0)
        marker_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("distance_markers", 1, true);
        obstacle_distances_sub_ = this->nh_.subscribe("/monitored_planning_scene", 1, &DebugObstacleDistance::obstacleDistancesCallback, this);
        pointCloud_obstacle_distances_sub_ = this->nh_.subscribe("/obstacle_distance_moveit/cam3d_left/filtered_cloud", 1, &DebugObstacleDistance::pointCloudObstacleDistancesCallback, this);


        ros::Duration(1.0).sleep();
        ROS_WARN("Debug initialized.");
        return 0;
    }

     void obstacleDistancesCallback(const moveit_msgs::PlanningScene::ConstPtr& msg)
    {
        //ROS_INFO_STREAM("fixed transformation size: "<<msg->fixed_frame_transforms.size());
        //ROS_INFO_STREAM(msg->name);

        for (size_t i = 0; i < msg->fixed_frame_transforms.size(); i=i+1)
        {     ;
            //ROS_INFO_STREAM(msg->fixed_frame_transforms[i].header);
            //ROS_INFO_STREAM(msg->fixed_frame_transforms[i].child_frame_id);
            //ROS_INFO_STREAM(msg->fixed_frame_transforms[i].transform.translation);
        }

    } 

    void pointCloudObstacleDistancesCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        visualization_msgs::MarkerArray marker_array;
        std::vector<double> dist_vec_link_1(1);
        geometry_msgs::PoseStamped dist_pose, min_dist_pose;
        double min_x = 1000.0,min_y = 1000.0,min_z = 1000.0;

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
        {     ;   
            //ROS_INFO_STREAM(cloud->points[i].x);
        }
        
        // visualization of collison distance with maker arrow        
        for (size_t i = 0; i < cloud->points.size (); i=i+1)
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
            geometry_msgs::Point end;
            end.x = cloud->points[i].x;
            end.y = cloud->points[i].y;
            end.z = cloud->points[i].z;

            // transform pose to other links of the arm            
            geometry_msgs::PoseStamped pose;
            pose.header = msg->header;
            pose.pose.position.x = end.x;      
            pose.pose.position.y = end.y;
            pose.pose.position.z = end.z;
            pose.pose.orientation.x = 0.0;      
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            dist_pose = transformPose("arm_4_link",pose);
            //publishTransformFrame(transformPose("arm_1_link",pose), std::string("pose_1"));

            
            const double dist = std::sqrt(dist_pose.pose.position.x*dist_pose.pose.position.x
                                        + dist_pose.pose.position.y*dist_pose.pose.position.y
                                        + dist_pose.pose.position.z*dist_pose.pose.position.z);
            
            //publishTransformFrame(dist_pose, "pose_"+std::to_string(2*i));

            if (dist <= min_x)
            {   
                min_x = dist;
                ROS_INFO("DISTNACE IS LESS THAN MIN_DIST");
                min_dist_pose = dist_pose;
            }
            

           //min_dist_pose.header = dist_pose.header;
           /*min_dist_pose.pose.orientation.x = 0.0;
           min_dist_pose.pose.orientation.y = 0.0;
           min_dist_pose.pose.orientation.z = 0.0;
           min_dist_pose.pose.orientation.w = 1.0;
            */
           /*min_dist_pose.pose.orientation = dist_pose.pose.orientation;

           if (dist_pose.pose.position.x <= min_x)
           {    
                min_x = dist_pose.pose.position.x;
                min_dist_pose.pose.position.x = dist_pose.pose.position.x;   
           }
           if (dist_pose.pose.position.y <= min_y)
           {
                min_y = dist_pose.pose.position.y;
                min_dist_pose.pose.position.y = dist_pose.pose.position.y;   
           }
           if (dist_pose.pose.position.z <= min_z)
           {
                min_z = dist_pose.pose.position.z;
                min_dist_pose.pose.position.z = dist_pose.pose.position.z;   
           }*/

            marker_vector.points.push_back(start);
            marker_vector.points.push_back(end);
            marker_array.markers.push_back(marker_vector);
        }
        publishTransformFrame(min_dist_pose, std::string("pose_1"));
        std::cout << "MIN_DISTANCE: " << min_x << std::endl;
        marker_pub_.publish(marker_array);
        //pcl::PointCloud<pcl::PointXYZ> cloud;
        //pcl::fromROSMsg(*msg, cloud);
        
    }

    geometry_msgs::PoseStamped transformPose(const std::string& target_frame, const geometry_msgs::PoseStamped& stamped_in)
    {
        ROS_INFO("KinematicUtils::transformPose ... transform pose from %s to %s", target_frame.c_str(),
        stamped_in.header.frame_id.c_str());
        bool transform = false;
        geometry_msgs::PoseStamped stamped_out;

        do
        {

            if (transform_listener_.frameExists(target_frame) &&
                transform_listener_.canTransform(target_frame, stamped_in.header.frame_id, stamped_in.header.stamp))
            {

                // wait for transformation from target (base_link) to goal frame
                transform_listener_.waitForTransform(target_frame, stamped_in.header.frame_id, stamped_in.header.stamp,
                                                    ros::Duration(10.0));
                transform_listener_.transformPose(target_frame, stamped_in, stamped_out);
                transform = true;
                ROS_INFO("KinematicUtils::transformPose: transformPose SUCCEED!!");

            }

            else
            {
                transform = false;
                ROS_ERROR_STREAM("KinematicUtils::transformPose" << target_frame.c_str() << " does not exist");
            }

        } while (!transform && ros::ok());

        return stamped_out;
    }


    void publishTransformFrame(const geometry_msgs::PoseStamped& pub_pose, const std::string& frame_id)
    {
    
        geometry_msgs::TransformStamped static_transformStamped;

        static_transformStamped.header.stamp = pub_pose.header.stamp;
        static_transformStamped.header.frame_id = pub_pose.header.frame_id;
        static_transformStamped.child_frame_id = frame_id;

        // translation
        static_transformStamped.transform.translation.x = pub_pose.pose.position.x;
        static_transformStamped.transform.translation.y = pub_pose.pose.position.y;
        static_transformStamped.transform.translation.z = pub_pose.pose.position.z;

        // rotation
        static_transformStamped.transform.rotation.x = pub_pose.pose.orientation.x;
        static_transformStamped.transform.rotation.y = pub_pose.pose.orientation.y;
        static_transformStamped.transform.rotation.z = pub_pose.pose.orientation.z;
        static_transformStamped.transform.rotation.w = pub_pose.pose.orientation.w;

        static_broadcaster.sendTransform(static_transformStamped);

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