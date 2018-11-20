#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from cob_control_msgs.srv import GetObstacleDistance, GetObstacleDistanceRequest, GetObstacleDistanceResponse

if __name__ == "__main__":
    rospy.wait_for_service('/arm/calculate_selfcollision_distance')
    try:
        client = rospy.ServiceProxy('/arm/calculate_selfcollision_distance', GetObstacleDistance)
        req = GetObstacleDistanceRequest()
        req.links.append("arm_1_link")
        req.links.append("arm_2_link")
        req.links.append("arm_3_link")
        req.links.append("arm_4_link")
        req.links.append("arm_5_link")
        req.links.append("arm_8_link")
        req.links.append("arm_7_link")
        res = client(req)
        print res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
