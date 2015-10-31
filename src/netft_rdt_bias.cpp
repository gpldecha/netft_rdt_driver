#include "netft_rdt_driver/netft_rdt_bias.h"

namespace netft_rdt_driver {

NetFTRDTDriverBias::NetFTRDTDriverBias(ros::NodeHandle& nh,std::size_t num_points):
num_points(num_points)
{
    count                = 0;
    force_b.x  = 0;
    force_b.y  = 0;
    force_b.z  = 0;

    torque_b.x = 0;
    torque_b.y = 0;
    torque_b.z = 0;

    service_server  = nh.advertiseService("bias_cmd",&NetFTRDTDriverBias::service_callback,this);
    bComputeBias    = false;

}

void NetFTRDTDriverBias::update(geometry_msgs::Wrench& wrench){

    if(!bComputeBias){
        wrench.force.x = wrench.force.x - force_b.x;
        wrench.force.y = wrench.force.y - force_b.y;
        wrench.force.z = wrench.force.z - force_b.z;

        wrench.torque.x = wrench.torque.x - torque_b.x;
        wrench.torque.y = wrench.torque.y - torque_b.y;
        wrench.torque.z = wrench.torque.z - torque_b.z;
    }
}

void NetFTRDTDriverBias::compute_bias(const geometry_msgs::Wrench& wrench){
    if(bComputeBias){
        if(count < num_points){
            force_b.x  = force_b.x + wrench.force.x;
            force_b.y  = force_b.y + wrench.force.y;
            force_b.z  = force_b.z + wrench.force.z;

            torque_b.x = torque_b.x + wrench.torque.x;
            torque_b.y = torque_b.y + wrench.torque.y;
            torque_b.z = torque_b.z + wrench.torque.z;
            count++;
        }else{

            force_b.x = (1/(double)num_points) * force_b.x;
            force_b.y = (1/(double)num_points) * force_b.y;
            force_b.z = (1/(double)num_points) * force_b.z;

            torque_b.x = (1/(double)num_points) * torque_b.x;
            torque_b.y = (1/(double)num_points) * torque_b.y;
            torque_b.z = (1/(double)num_points) * torque_b.z;
            print_bias();
            bComputeBias = false;
        }
    }
}

bool NetFTRDTDriverBias::service_callback(netft_rdt_driver::String_cmd::Request& request, netft_rdt_driver::String_cmd::Response& response){
    std::string cmd = request.cmd;

    if(cmd == "bias"){
        bComputeBias=true;
        force_b.x  = force_b.y  = force_b.z  = 0;
        torque_b.x = torque_b.y = torque_b.z = 0;
        count = 0;
        response.res = " command [" + cmd + "] sucessfully called";
        return true;
    }else if (cmd == "print"){
        response.res = "commands : bias | print";
        return true;
    }else{
       std::string res =  "no such cmd [" +  cmd +  "] defined        NetFTRDTDriverBias::service_callback";
       response.res = res;
       return false;
    }

}

void NetFTRDTDriverBias::print_bias() const{
    std::cout<< "=== bias (mean) ===" <<std::endl;
    std::cout<< "F:         " << force_b.x << "\t" << force_b.y << "\t"   << force_b.z << std::endl;
    std::cout<< "T:         " << torque_b.x << "\t" << torque_b.y << "\t" << torque_b.z << std::endl;
    std::cout<< "nbSamples: " << num_points << std::endl;
}


}
