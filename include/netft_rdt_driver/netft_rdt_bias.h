#ifndef NETFT_RDT_BIAS_H_
#define NETFT_RDT_BIAS_H_

#include <ros/ros.h>

#include <geometry_msgs/Wrench.h>

#include "netft_rdt_driver/String_cmd.h"


namespace netft_rdt_driver
{

class NetFTRDTDriverBias{

public:

    NetFTRDTDriverBias(ros::NodeHandle& nh,std::size_t num_points=200);

    void update(geometry_msgs::Wrench& wrench);

    void compute_bias(const geometry_msgs::Wrench& wrench);

private:

    void print_bias() const;

    bool service_callback(netft_rdt_driver::String_cmd::Request& request, netft_rdt_driver::String_cmd::Response& response);

private:


    geometry_msgs::Vector3  force_b;
    geometry_msgs::Vector3  torque_b;

    ros::ServiceServer    service_server;

    std::size_t           num_points;
    std::size_t           count;
    bool                  bComputeBias;

};

}


#endif
