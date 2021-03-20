#include <handle_sim.h>

handle_sim::handle_sim(){
    hand_open_sprd_srv  = nh.advertiseService("open_spread",  &handle_sim::handOpenSpread, this);
    hand_close_sprd_srv = nh.advertiseService("close_spread", &handle_sim::handCloseSpread, this);
    hand_grsp_pos_srv   = nh.advertiseService("grasp_pos",    &handle_sim::handGraspPos, this);

    //bhand_joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    j11_pub = nh.advertise<std_msgs::Float64>("bh_j11_position_controller/command", 1);
    j21_pub = nh.advertise<std_msgs::Float64>("bh_j21_position_controller/command", 1);
    j12_pub = nh.advertise<std_msgs::Float64>("bh_j12_position_controller/command", 1);
    j22_pub = nh.advertise<std_msgs::Float64>("bh_j22_position_controller/command", 1);
    j32_pub = nh.advertise<std_msgs::Float64>("bh_j32_position_controller/command", 1);

}

handle_sim::~handle_sim() {}

bool handle_sim::handOpenSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    //ROS_INFO("Spread Open Request");
    std_msgs::Float64 msg;
    msg.data = 0;

    j11_pub.publish(msg);
    j21_pub.publish(msg);
    return true;
}

bool handle_sim::handCloseSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    //ROS_INFO("Spread Close Request");
    std_msgs::Float64 msg;
    msg.data = 3.1456;

    j11_pub.publish(msg);
    j21_pub.publish(msg);
    return true;
}

bool handle_sim::handGraspPos(wam_srvs::BHandGraspPos::Request &req, wam_srvs::BHandGraspPos::Response &res) {
    //ROS_INFO("Grasp Position Request %f", req.radians);
    std_msgs::Float64 msg;
    msg.data = req.radians;

    j12_pub.publish(msg);
    j22_pub.publish(msg);
    j32_pub.publish(msg);
    return true;
}

int main(int argc, char **argv){
    ros::init(argc,argv, "handle_sim");
    ROS_INFO("Node Started...");
    handle_sim hSim;
    ros::Rate loop_rate(200);
    ros::spin();

    return 0;
}
