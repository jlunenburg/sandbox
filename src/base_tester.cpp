/** Publishes cmd_vels to test the performance of the base */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

/// Logging data
#include <tinyxml.h>
#include <stdlib.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "base_performance_tester");
    ros::NodeHandle nh;

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::ServiceClient amcl_srv = nh.serviceClient<std_srvs::Empty>("/request_nomotion_update");

    std::string filename("../parameters/base_trajectory.xml");

    TiXmlDocument doc(filename.c_str());
    bool loadOkay = doc.LoadFile();
    if (loadOkay) {
        printf("Sucessfully loaded %s:\n", filename.c_str());
    } else {
        printf("Failed to load file \"%s\"\n", filename.c_str());
    }

    TiXmlElement* pathdescription = doc.FirstChildElement();

    /// Loop over all sections
    geometry_msgs::Twist cmd_vel;
    std_srvs::Empty srv;
    for(TiXmlElement* section = pathdescription->FirstChildElement(); section != NULL; section = section->NextSiblingElement())
    {
    /// Publish zero and call amcl (convenient for analysis)
	cmd_vel_pub.publish(cmd_vel);
    amcl_srv.call(srv);

        /// Get ID
        std::string id(section->Attribute("id"));

        /// Get Duration
        std::string sduration(section->Attribute("duration"));
        ros::Duration duration(std::atof(sduration.c_str()));
        /// Fill message

        TiXmlElement* twist  = section->FirstChildElement( "Twist");
        if (twist == NULL) {
            ROS_ERROR("Cannot read twist of section %s", id.c_str());
        } else {
            TiXmlElement* linear = twist->FirstChildElement( "linear" );
            if (linear == NULL) {
                ROS_ERROR("Cannot read linear velocity of section %s", id.c_str());
            } else {
                cmd_vel.linear.x = atof(linear->Attribute("x"));
                cmd_vel.linear.y = atof(linear->Attribute("y"));
            }

            TiXmlElement* angular = twist->FirstChildElement( "angular" );
            if (angular == NULL) {
                ROS_ERROR("Cannot read angular velocity of section %s", id.c_str());
            } else {
                cmd_vel.angular.z = atof(angular->Attribute("z"));
            }
        }

        ROS_INFO("Section %s: [%f, %f, %f] with duration %f", id.c_str(), cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, duration.toSec());

        /// Publish cmd_vel
        ros::Time starttime = ros::Time::now();
        while ( (ros::Time::now() - starttime).toSec() < duration.toSec() ) {
            cmd_vel_pub.publish(cmd_vel);
            amcl_srv.call(srv);
            ros::Duration(0.001).sleep();// Assuming the service call is slow enough
        }

        /// Stop robot
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub.publish(cmd_vel);
        amcl_srv.call(srv);

        /// Wait 2 seconds for the robot to come to a stop before starting the next section
        starttime = ros::Time::now();
        duration = ros::Duration(2.0);
        while ( (ros::Time::now() - starttime).toSec() < duration.toSec() ) {
            amcl_srv.call(srv);
            ros::Duration(0.001).sleep();// Assuming the service call is slow enough
        }
        //ros::Duration(2.0).sleep();

    }

}
