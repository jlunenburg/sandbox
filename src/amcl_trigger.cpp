
/** Publishes cmd_vels to test the performance of the base */

#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "amcl_trigger");
    ros::NodeHandle nh;

    ros::ServiceClient amcl_srv = nh.serviceClient<std_srvs::Empty>("/request_nomotion_update");

    std_srvs::Empty srv;
    
    ros::Rate r(40);
    
    while (nh.ok()) {
        amcl_srv.call(srv);
        r.sleep();
    }

}
