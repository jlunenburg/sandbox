#! /usr/bin/env python
import roslib; roslib.load_manifest('reachability_analysis')
import rospy

import sys
import os

# Visualization
import visualization_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

# XML
import xml.etree.ElementTree as ET

if __name__ == '__main__':

    rospy.init_node('reachability_data_analyzer')
    rospy.loginfo("Node initialized")
    
    marker_array_pub = rospy.Publisher('/visualization_marker', visualization_msgs.msg.Marker)
    
    ''' Load data '''
    root = None
    for id in range(0, int(sys.argv[3])+1):
        filename = os.environ["HOME"]+"/ros/data/recorded/rosbags/reachability_analysis/"+sys.argv[1]+"/"+sys.argv[2]+format(id)+".xml"
        rospy.loginfo("Filename = {0}".format(filename))
        try:
            tree = ET.parse(filename)
        except:
           rospy.logwarn("Cannot open file '{0}',\n  Usage: rosrun reachability_analysis AnalyzeData.py YYYYMMDD robot nr_ids\nExample: rosrun reachability_analysis AnalyzeData.py 20140731 sergio 8".format(filename))
           sys.exit(0)
        
        if root == None:
            root = tree.getroot()
        else:
            root.extend(tree.getroot())
            
    ''' Get nr of poses per sphere and the maximum feasible number '''
    nr_poses_sphere = root.get("number_points_sphere")
    rospy.loginfo("Number of poses per sphere = {0}".format(nr_poses_sphere))
    
    max_feasible_poses = 0
    min_feasible_poses = 100000
    
    total_feasible_poses = 0
    total_nr_feasible_spheres = 0
    
    minx = float( 1000)
    maxx = float(-1000)
    miny = float( 1000)
    maxy = float(-1000)
    minz = float( 1000)
    maxz = float(-1000)
    
    for sphere in root:
        nr_feasible_poses = float(sphere.get("FeasibleNrPoses"))
        max_feasible_poses = max(max_feasible_poses, nr_feasible_poses)
        if nr_feasible_poses > 0:
            min_feasible_poses = min(min_feasible_poses, nr_feasible_poses)
            
            minx = min(minx,float(sphere.get("x")))
            maxx = max(maxx,float(sphere.get("x")))
            miny = min(miny,float(sphere.get("y")))
            maxy = max(maxy,float(sphere.get("y")))
            minz = min(minz,float(sphere.get("z")))
            maxz = max(maxz,float(sphere.get("z")))
            
    rospy.loginfo("Minimum number of feasible poses = {0}".format(min_feasible_poses))
    rospy.loginfo("Maximum number of feasible poses = {0}".format(max_feasible_poses))
    
    ''' Put in marker array '''
    marker = visualization_msgs.msg.Marker()
    marker.header.frame_id = root[0].get("frame_id")
    marker.header.stamp = rospy.Time.now()
    marker.type = visualization_msgs.msg.Marker.SPHERE_LIST
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = float(root.get("stepsize"))
    marker.scale.y = float(root.get("stepsize"))
    marker.scale.z = float(root.get("stepsize"))
    for sphere in root:
        point = geometry_msgs.msg.Point()
        point.x = float(sphere.get("x"))
        point.y = float(sphere.get("y"))
        point.z = float(sphere.get("z"))
        
        

        # For angle
        if (point.z < 1.0 or (point.x < 0.0 and point.y > 0.2) ):
        # For side view
        #if ((point.x*point.x + point.y*point.y) > 0.1225):
        # For top view
        #if (marker.pose.position.z < 1.0 and (marker.pose.position.x*marker.pose.position.x + marker.pose.position.y*marker.pose.position.y) > 0.1225):
            ''' Assign new values (see http://en.wikipedia.org/wiki/HSL_and_HSV for inspiration) '''
            rate = float(sphere.get("FeasibleNrPoses"))/float(sphere.get("TotalNrPoses"))
            if rate > 0.001:
                color = std_msgs.msg.ColorRGBA()
                if rate < 0.25:
                    color.r = 1.0
                    color.g = rate/0.25
                    color.b = 0.0
                elif (rate >= 0.25 and rate < 0.5):
                    color.r = 1.0 - (rate-0.25)/0.25
                    color.g = 1.0
                    color.b = 0.0
                elif (rate >= 0.5 and rate < 0.75):
                    color.r = 0.0
                    color.g = 1.0
                    color.b = (rate-0.5)/0.25
                elif (rate >= 0.75 and rate <= 1.0):
                    color.r = 0.0
                    color.g = 1.0 - (rate-0.75)/0.25
                    color.b = 1.0
                else:
                    rospy.logwarn("Rate = {0}, don't know which color".format(rate))
                color.a = 1.0
                
                marker.points.append(point)
                marker.colors.append(color)
                
                total_feasible_poses = total_feasible_poses + float(sphere.get("FeasibleNrPoses"))
                total_nr_feasible_spheres = total_nr_feasible_spheres +1
    
    ''' Publish marker ''' 
    rospy.sleep(1.0)
    marker_array_pub.publish(marker)
    rospy.loginfo("Analysis succeeded")
    rospy.loginfo("x: min, max = {0}, {1}".format(minx, maxx))
    rospy.loginfo("y: min, max = {0}, {1}".format(miny, maxy))
    rospy.loginfo("z: min, max = {0}, {1}".format(minz, maxz))
    rospy.loginfo("Number of feasible spheres = {0}".format(total_nr_feasible_spheres))
    rospy.loginfo("Number of feasible poses = {0}".format(total_feasible_poses))
    
    
