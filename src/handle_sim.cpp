#include <handle_sim.h>

handle_sim::handle_sim() : joint_pos({0,0,0,0,0}) {

    joint_sub = nh.subscribe("/joint_states",100,&handle_sim::jointStateCallback,this);
    joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states_sim", 1);

    j11_pub = nh.advertise<std_msgs::Float64>("bh_j11_position_controller/command", 1);
    j21_pub = nh.advertise<std_msgs::Float64>("bh_j21_position_controller/command", 1);
    j12_pub = nh.advertise<std_msgs::Float64>("bh_j12_position_controller/command", 1);
    j22_pub = nh.advertise<std_msgs::Float64>("bh_j22_position_controller/command", 1);
    j32_pub = nh.advertise<std_msgs::Float64>("bh_j32_position_controller/command", 1);

    hand_grsp_pos_srv   = nh.advertiseService("grasp_pos",    &handle_sim::handGraspPos, this);
    hand_sprd_pos_srv   = nh.advertiseService("spread_pos",   &handle_sim::handSpreadPos, this);
    hand_open_sprd_srv  = nh.advertiseService("open_spread",  &handle_sim::handOpenSpread, this);
    hand_close_sprd_srv = nh.advertiseService("close_spread", &handle_sim::handCloseSpread, this);
}

handle_sim::~handle_sim() {}

void handle_sim::jointStateCallback(const sensor_msgs::JointState& msg) {

    joint_pos.j11 = msg.position[0];
    joint_pos.j21 = msg.position[3];
    joint_pos.j12 = msg.position[1];
    joint_pos.j22 = msg.position[4];
    joint_pos.j32 = msg.position[6];

    sensor_msgs::JointState bhand_joint_state;

    const char* bhand_jnts[] = {"wam/BHand/FingerOne/KnuckleTwoJoint",
                                "wam/BHand/FingerTwo/KnuckleTwoJoint",
                                "wam/BHand/FingerThree/KnuckleTwoJoint",
                                "wam/BHand/FingerOne/KnuckleOneJoint",
                                "wam/BHand/FingerTwo/KnuckleOneJoint",
                                "wam/BHand/FingerOne/KnuckleThreeJoint",
                                "wam/BHand/FingerTwo/KnuckleThreeJoint",
                                "wam/BHand/FingerThree/KnuckleThreeJoint"};
    std::vector <std::string> bhand_joints(bhand_jnts, bhand_jnts + 8);
    bhand_joint_state.name.resize(8);
    bhand_joint_state.name = bhand_joints;
    bhand_joint_state.position.resize(8);

    bhand_joint_state.position[0] = joint_pos.j12;
    bhand_joint_state.position[1] = joint_pos.j22;
    bhand_joint_state.position[2] = joint_pos.j32;
    bhand_joint_state.position[3] = joint_pos.j11;
    bhand_joint_state.position[4] = 0; // Not testing in sim
    bhand_joint_state.position[5] = 0; // "
    bhand_joint_state.position[6] = 0; // "
    bhand_joint_state.position[7] = 0; // "

    joint_pub.publish(bhand_joint_state);
}

bool handle_sim::handOpenSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ROS_INFO("Spread Open Request");
    std_msgs::Float64 msg;
    msg.data = 0;

    j11_pub.publish(msg);
    j21_pub.publish(msg);
    return true;
}

bool handle_sim::handCloseSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    //ROS_INFO("Spread Close Request");
    std_msgs::Float64 msg1, msg2;
    msg1.data = (3.1456 < obstacle.j11) ? 3.1456 : obstacle.j11;
    msg2.data = (3.1456 < obstacle.j21) ? 3.1456 : obstacle.j21;

    j11_pub.publish(msg1);
    j21_pub.publish(msg2);
    return true;
}

bool handle_sim::handSpreadPos(wam_srvs::BHandSpreadPos::Request &req, wam_srvs::BHandSpreadPos::Response &res) {
    //ROS_INFO("Grasp Position Request %f", req.radians);

    std_msgs::Float64 msg1, msg2;
    msg1.data = (req.radians < obstacle.j11) ? req.radians : obstacle.j11;
    msg2.data = (req.radians < obstacle.j21) ? req.radians : obstacle.j21;

    j11_pub.publish(msg1);
    j21_pub.publish(msg2);

    return true;
}

bool handle_sim::handGraspPos(wam_srvs::BHandGraspPos::Request &req, wam_srvs::BHandGraspPos::Response &res) {
    //ROS_INFO("Grasp Position Request %f", req.radians);
    std_msgs::Float64 msg1, msg2, msg3;
    msg1.data = (req.radians < obstacle.j12) ? req.radians : obstacle.j12;
    msg2.data = (req.radians < obstacle.j22) ? req.radians : obstacle.j22;
    msg3.data = (req.radians < obstacle.j32) ? req.radians : obstacle.j32;

    j12_pub.publish(msg1);
    j22_pub.publish(msg2);
    j32_pub.publish(msg3);
    return true;
}

void handle_sim::start() {
    ROS_INFO("Node Started...");

    // Please set good obstacle values. I don't have time for error correction.
    nh.param<double>("/handle_sim/j11", obstacle.j11, 3.14157);
    nh.param<double>("/handle_sim/j21", obstacle.j21, 3.14157);
    nh.param<double>("/handle_sim/j12", obstacle.j12, 3.14157);
    nh.param<double>("/handle_sim/j22", obstacle.j22, 3.14157);
    nh.param<double>("/handle_sim/j32", obstacle.j32, 3.14157);

    ros::spin();
}

int main(int argc, char **argv){
    try {
        ros::init(argc, argv, "handle_sim");
        handle_sim hSim;
        hSim.start();
    }catch(std::exception &e){
        std::cout<< "\033[1;31m[ERROR] An exception was caught, with message: ";
        std::cout<< e.what() << std::endl;
    }

    return 0;
}
