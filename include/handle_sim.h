#ifndef SRC_HANDLE_SIM_H
#define SRC_HANDLE_SIM_H

#include <ros/ros.h>
#include "wam_srvs/BHandGraspPos.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

class handle_sim {
    public:
    handle_sim();
    ~handle_sim();
    void run();

    private:
    ros::NodeHandle nh;

    sensor_msgs::JointState bhand_joint_state;
    ros::Publisher bhand_joint_state_pub;
    ros::Publisher j11_pub;
    ros::Publisher j21_pub;
    ros::Publisher j12_pub;
    ros::Publisher j22_pub;
    ros::Publisher j32_pub;

    // Services taken from wam_bringup-master bhand_node.h
    ros::ServiceServer hand_open_sprd_srv;
    ros::ServiceServer hand_close_sprd_srv;
    ros::ServiceServer hand_grsp_pos_srv;

    bool handOpenSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool handCloseSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool handGraspPos(wam_srvs::BHandGraspPos::Request &req, wam_srvs::BHandGraspPos::Response &res);
};


#endif //SRC_HANDLE_SIM_H
