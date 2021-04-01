#ifndef SRC_HANDLE_SIM_H
#define SRC_HANDLE_SIM_H

#include <ros/ros.h>
#include "std_srvs/Empty.h"
#include "std_msgs/Float64.h"
#include "wam_srvs/BHandSpreadPos.h"
#include "wam_srvs/BHandGraspPos.h"
#include "sensor_msgs/JointState.h"

// A struct is less elegant but makes the code more readable.
struct jointStruct{ double j11,j21,j12,j22,j32; };

class handle_sim {
    public:
    handle_sim();
    ~handle_sim();
    void start();

    private:
    jointStruct obstacle; // 0 corresponds to no obstacle, 2.1223 is full obstacle
    jointStruct joint_pos;

    ros::NodeHandle nh;

    sensor_msgs::JointState bhand_joint_state;
    ros::Subscriber joint_sub;
    ros::Publisher joint_pub;

    ros::Publisher j11_pub;
    ros::Publisher j21_pub;
    ros::Publisher j12_pub;
    ros::Publisher j22_pub;
    ros::Publisher j32_pub;

    // Services taken from wam_bringup-master bhand_node.h
    ros::ServiceServer hand_open_sprd_srv;
    ros::ServiceServer hand_close_sprd_srv;
    ros::ServiceServer hand_sprd_pos_srv;
    ros::ServiceServer hand_grsp_pos_srv;

    void jointStateCallback(const sensor_msgs::JointState&);

    bool handOpenSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool handCloseSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool handSpreadPos(wam_srvs::BHandSpreadPos::Request &req, wam_srvs::BHandSpreadPos::Response &res);
    bool handGraspPos(wam_srvs::BHandGraspPos::Request &req, wam_srvs::BHandGraspPos::Response &res);
};


#endif //SRC_HANDLE_SIM_H
