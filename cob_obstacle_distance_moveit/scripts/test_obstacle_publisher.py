#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rospy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh

object_position = [
    [0.4606, -0.3235, 0.646795],
    [0.51, -0.07, 0.77],
    [-0.26, -0.29, 0.89],
    [0.71, -0.18, 0.31]
]


if __name__ == "__main__":
    rospy.init_node("simple_obstacle_pub")
    root_frame = "world"

    pub = rospy.Publisher("/collision_object", CollisionObject, queue_size = 1000)
    pub1 = rospy.Publisher("/arm/obstacle_distance/registerObstacle", CollisionObject, queue_size = 1)
    rospy.sleep(1.0)

    # Publish a simple sphere
    x = CollisionObject()
    x.id = "sphere"
    x.header.frame_id = root_frame
    x.operation = CollisionObject.ADD
    #x.operation = CollisionObject.REMOVE
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.BOX
    sphere.dimensions.append(0.1)  # radius
    sphere.dimensions.append(0.1)  # radius
    sphere.dimensions.append(0.1)  # radius
    x.primitives.append(sphere)

    pose = Pose()
    pose.position.x = 0.4606
    pose.position.y = -0.3235
    pose.position.z = 0.646795
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    x.primitive_poses.append(pose)
    pub.publish(x)
    pub1.publish(x)
    rospy.sleep(5.0)

#    # Publish a simple sphere
#    y = CollisionObject()
#    y.id = "sphere1"
#    y.header.frame_id = root_frame
#    y.operation = CollisionObject.ADD
#    #x.operation = CollisionObject.REMOVE
#    sphere = SolidPrimitive()
#    sphere.type = SolidPrimitive.BOX
#    sphere.dimensions.append(0.1)  # radius
#    sphere.dimensions.append(0.1)  # radius
#    sphere.dimensions.append(0.1)  # radius
#    y.primitives.append(sphere)

#    pose = Pose()
#    pose.position.x = 0.51
#    pose.position.y = -0.07
#    pose.position.z = 0.77
#    pose.orientation.x = 0.0;
#    pose.orientation.y = 0.0;
#    pose.orientation.z = 0.0;
#    pose.orientation.w = 1.0;
#    y.primitive_poses.append(pose)
#    pub.publish(y)
#    pub1.publish(y)
#    rospy.sleep(5.0)

#    # Publish a simple sphere
#    z = CollisionObject()
#    z.id = "sphere3"
#    z.header.frame_id = root_frame
#    z.operation = CollisionObject.ADD
#    #x.operation = CollisionObject.REMOVE
#    sphere = SolidPrimitive()
#    sphere.type = SolidPrimitive.BOX
#    sphere.dimensions.append(0.1)  # radius
#    sphere.dimensions.append(0.1)  # radius
#    sphere.dimensions.append(0.1)  # radius
#    z.primitives.append(sphere)

#    pose = Pose()
#    pose.position.x = -0.26
#    pose.position.y = -0.29
#    pose.position.z = 0.89
#    pose.orientation.x = 0.0;
#    pose.orientation.y = 0.0;
#    pose.orientation.z = 0.0;
#    pose.orientation.w = 1.0;
#    z.primitive_poses.append(pose)
#    pub.publish(z)
#    pub1.publish(z)
#    rospy.sleep(5.0)

#    # Publish a simple sphere
#    a = CollisionObject()
#    a.id = "sphere4"
#    a.header.frame_id = root_frame
#    a.operation = CollisionObject.ADD
#    #x.operation = CollisionObject.REMOVE
#    sphere = SolidPrimitive()
#    sphere.type = SolidPrimitive.BOX
#    sphere.dimensions.append(0.1)  # radius
#    sphere.dimensions.append(0.1)  # radius
#    sphere.dimensions.append(0.1)  # radius
#    a.primitives.append(sphere)

#    pose = Pose()
#    pose.position.x = 0.71
#    pose.position.y = -0.18
#    pose.position.z = 0.31
#    pose.orientation.x = 0.0;
#    pose.orientation.y = 0.0;
#    pose.orientation.z = 0.0;
#    pose.orientation.w = 1.0;
#    a.primitive_poses.append(pose)
#    pub.publish(a)
#    pub1.publish(a)
#    rospy.sleep(5.0)

    rospy.loginfo("Done")
