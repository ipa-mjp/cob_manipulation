/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
 
#ifndef COB_PICK_ACTION_H
#define COB_PICK_ACTION_H

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
//#include <geometry_msgs/Quaternion.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>
#include <moveit_msgs/Grasp.h>


#include <cob_grasp_generation/QueryGraspsAction.h>
#include <ipa_manipulation_msgs/GraspPoseAction.h>
#include <GraspTable.h>



class CobGraspActionServer
{
private:
	ros::NodeHandle nh_;

	ros::Publisher pub_co; //publisher for collision_objects
	ros::Publisher pub_ao; //publisher for attached_collision_objects

	boost::scoped_ptr<actionlib::SimpleActionClient<cob_grasp_generation::QueryGraspsAction> > ac_grasps_or;
	boost::scoped_ptr<actionlib::SimpleActionServer<ipa_manipulation_msgs::GraspPoseAction> > grasp_server_;

	char* GraspTableIniFile;
	GraspTable* m_GraspTable;

	bool last_grasp_valid;
	std::string last_object_name;

	tf::TransformListener tf_listener_;
	tf::TransformBroadcaster tf_broadcaster_;

	std::map<unsigned int,std::string> map_classid_to_classname;

	std::string manipulation_group_name, end_effector_link;

public:
	CobGraspActionServer(std::string group_name): manipulation_group_name(group_name){
		if(!nh_.getParam(ros::this_node::getName() +"/end_effector_name", end_effector_link))
		{
			end_effector_link = "gripper_left_base_link";
			ROS_ERROR("ArmPlanner::initialize --> No end_effector_name available on parameter server");
		}
	};

	~CobGraspActionServer();

	void initialize();
	void run();

	void grasp_generation_cb(const ipa_manipulation_msgs::GraspPoseGoalConstPtr &goal);

	void insertObject(std::string object_name, unsigned int object_class, geometry_msgs::PoseStamped object_pose);

	void fillAllGraspsKIT(unsigned int objectClassId, std::string gripper_type, geometry_msgs::PoseStamped object_pose, std::vector<moveit_msgs::Grasp> &grasps);
	void fillSingleGraspKIT(unsigned int objectClassId, std::string gripper_type, unsigned int grasp_id, geometry_msgs::PoseStamped object_pose, std::vector<moveit_msgs::Grasp> &grasps);
	void convertGraspKIT(Grasp* current_grasp, geometry_msgs::PoseStamped object_pose, std::vector<moveit_msgs::Grasp> &grasps);

	void fillGraspsOR(unsigned int objectClassId, std::string gripper_type, std::string gripper_side, unsigned int grasp_id, geometry_msgs::PoseStamped object_pose, std::vector<moveit_msgs::Grasp> &grasps);
	//void fillGraspsOR(unsigned int objectClassId, std::string gripper_type, unsigned int grasp_id, geometry_msgs::PoseStamped object_pose, std::vector<moveit_msgs::Grasp> &grasps);

	trajectory_msgs::JointTrajectory MapHandConfiguration(sensor_msgs::JointState table_config);
	tf::Transform transformPose(tf::Transform transform_O_from_SDH, tf::Transform transform_HEADER_from_OBJECT, std::string object_frame_id);
	moveit_msgs::GripperTranslation calculateApproachDirection(geometry_msgs::Pose msg_pose_grasp_FOOTPRINT_from_ARM7, geometry_msgs::Pose msg_pose_pre_FOOTPRINT_from_ARM7);

};
#endif


