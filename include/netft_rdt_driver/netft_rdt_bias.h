#ifndef NETFT_RDT_BIAS_H_
#define NETFT_RDT_BIAS_H_

#include <geometry_msgs/Wrench.h>


namespace netft_rdt_driver
{

class NetFTRDTDriverBias{

public:

    NetFTRDTDriverBias(unsigned int num_points=200);

    void update(geometry_msgs::Wrench& wrench);

    void compute_bias(const geometry_msgs::Wrench& wrench);

private:

    void print_bias();

private:


    geometry_msgs::Vector3  force_b;
    geometry_msgs::Vector3  torque_b;

    unsigned int          num_points;
    unsigned int          count;
    bool                  bComputeBias;

};

}


#endif
