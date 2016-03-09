//
// Created by ptirindelli on 8/02/16.
//

#include "wolf_gps_node.h"


int main(int argc, char **argv)
{
    std::cout << "\n=========== WOLF GPS MAIN ===========\n\n";

    //init ros
    ros::init(argc, argv, "wolf_gps_node");

    /*
     * Parameters, to be optimized
     */
    StateBlock* sensor_p = new StateBlock(Eigen::Vector3s::Zero()); //gps sensor position
    sensor_p->fix(); // TODO only for now, to simplify things
    StateBlock* sensor_o = new StateBlock(Eigen::Vector4s::Zero(), ST_QUATERNION);   //gps sensor orientation
    sensor_o->fix(); //orientation is fixed, because antenna omnidirectional, so is not going to be optimized
    StateBlock* sensor_bias = new StateBlock(Eigen::Vector1s::Zero());    //gps sensor bias

    StateBlock* init_vehicle_p = new StateBlock(Eigen::Vector3s::Zero());    //vehicle initial position
    StateBlock* init_vehicle_o = new StateBlock(Eigen::Vector1s::Zero());// vehicle initial orientation

    SensorGPS* gps_sensor_ptr = new SensorGPS(sensor_p, sensor_o, sensor_bias, init_vehicle_p, init_vehicle_o);

    Eigen::Vector3s prior = Eigen::Vector3s::Zero();
    prior[0] = 4789360;
    prior[1] = 177175;
    prior[2] = 4194534;


            // Trilateration node
    WolfGPSNode wgNode(gps_sensor_ptr,
                       PO_2D,
                       nullptr,                           //_sensor_prior_ptr
                       Eigen::Vector3s::Zero(),           //prior
                       Eigen::Matrix3s::Identity()*0.01,  //prior cov
                       5,                                 //window size
                       1);                                //time for new keyframe

    ros::spin();

    return 0;
}