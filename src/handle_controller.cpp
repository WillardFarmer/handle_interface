#include <handle_controller.h>
#include <std_msgs/Float64.h>

handle_controller::handle_controller(const char *port) :
        handle_led_set(false),
        hand_state_set(false),
        hand_tgt_set(false),
        hand_lock(false),
        haptic_effort(0),
        //spread_pos(0),
        last_button_joy(false),
        last_button_push(false),
        timer_joy(0.5),
        timer_push(0.5),
        filtered_trigger(5),
        filtered_joy1(5),
        filtered_joy2(5),
        test(0){

    this->open_port(port);

    // For data visualisation
    hand_tgt_pub = nh.advertise<std_msgs::Float64MultiArray>("/hand_target", 10);
    handle_haptic_effort = nh.advertise<std_msgs::Float64>("/handle_haptic_effort", 10);
    handle_state_pub = nh.advertise<handle_interface::handle_state>("/handle_state", 10);
    hand_state_sub = nh.subscribe("joint_states_sim", 1,
                                  &handle_controller::hand_pos_callback, this);

    grasp_client = nh.serviceClient<wam_srvs::BHandGraspPos>("grasp_pos");
    spread_client = nh.serviceClient<wam_srvs::BHandSpreadPos>("spread_pos");
    spread_open_client = nh.serviceClient<std_srvs::Empty>("open_spread");
    spread_close_client = nh.serviceClient<std_srvs::Empty>("close_spread");

    pb_fil = nh.advertise<std_msgs::Float64>("/handle_fil",1);
    pb_raw = nh.advertise<std_msgs::Float64>("/handle_raw",1);

}

handle_controller::~handle_controller() {
    //Turn off LED and Haptic Feedback
    struct can_frame can_msg;
    can_msg.can_id = HANDLE_ID;
    can_msg.can_dlc = 8;
    can_msg.data[0] = 0x00;
    can_msg.data[1] = 0x00;
    can_msg.data[2] = 0x00;
    can_msg.data[3] = 0x00;
    can_msg.data[4] = 0x00;
    can_msg.data[5] = 0x00;
    can_msg.data[6] = 0x00;
    can_msg.data[7] = 0x00;

    send_port(&can_msg);
    close_port();
}

void handle_controller::open_port(const char *port) {
    ROS_INFO("Opening CAN port");

    struct ifreq ifr;
    struct sockaddr_can addr;

    /* open socket */
    soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(soc < 0) {
        throw std::runtime_error("Failed to obtain socket.");
    }

    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name, port);
    if(ioctl(soc, SIOCGIFINDEX, &ifr) < 0) {
        throw std::runtime_error("Could not retrieve interface index.");
    }

    addr.can_ifindex = ifr.ifr_ifindex;
    fcntl(soc, F_SETFL, O_NONBLOCK);
    if(bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        throw std::runtime_error("Could not bind socket to CAN interface.");
    }
}

void handle_controller::send_port(struct can_frame *frame)
{
    int retval;
    retval = write(soc, frame, sizeof(struct can_frame));
    if (retval != sizeof(struct can_frame))
    {
        throw std::runtime_error("Could not write to port.");
    }
    else
    {
        ROS_DEBUG("CAN Message successfully sent.");
    }
}

bool handle_controller::read_port() {
    ROS_DEBUG("Reading Port");
    struct can_frame frame_rd;
    int recvbytes = 0;

    int i = 0;
    bool read_can_port = true;
    do{
        i += 1;
        struct timeval timeout = {0, 10000};
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(soc, &readSet);

        if (select((soc + 1), &readSet, NULL, NULL, &timeout) >= 0) {
            if (FD_ISSET(soc, &readSet)) {
                recvbytes = read(soc, &frame_rd, sizeof(struct can_frame));
                if (recvbytes > 0) {
                    ROS_DEBUG("CAN message Found");
                    unpack_reply(&frame_rd);
                    read_can_port = false;
                }
            }
        }
    } while((i<3) && read_can_port && ros::ok());
    return !read_can_port;
}


void handle_controller::close_port() {
    ROS_INFO("Closing CAN Port.");
    close(soc);
}

/// JOINT STATE STRUCTURE //
/// Values are approximated from finger motor encoder counts
/// 0: Finger 1 [rad]
/// 1: Finger 2 [rad]
/// 2: Finger 3 [rad]
/// 3: Spread   [rad]
/// 4: Spread   (copied from 3) [rad]
/// 5: Finger 1 (second link)   [rad]
/// 6: Finger 2 (second link)   [rad]
/// 7: Finger 3 (second link)   [rad]
///
/// Close Target:
///     finger: 17,000 = ~2.1223rad 121.5988deg
///     spread:  3,150 = ~3.1456rad 180.2293deg
///
/// Open Target:
///     finger:      0 = 0
///     spread:      0 = 0
void handle_controller::hand_pos_callback(const sensor_msgs::JointState &msg) {
    ROS_DEBUG("Received Barrett Hand Pos message.");

    /*
    std_msgs::Float64 a; a.data = hand_tgt_pos[0];
    std_msgs::Float64 b; b.data = msg.position[0];

    pb_fil.publish(a);
    pb_raw.publish(b);
     */

    try {
        // Very basic haptic logic. Will need further development.
        if (hand_tgt_set) {
            double max_diff = 0;

            for (int i = 0; i < 3; i++) {
                // If finger has not moved. 31*100 * 2.1223/17000 * 1/10 = 0.0387 rad/pub
                // This number is the angle a finger should have moved since the last pos message.
                // * 0.1 is a scaling factor to ensure the finger is stopped or very slow.
                if( abs(hand_state[i] - msg.position[i]) < (0.0387 * 0.1) ) {
                    if (hand_tgt_pos[i] < msg.position[i]) continue;
                    double diff = (hand_tgt_pos[i] - msg.position[i]);
                    max_diff = diff > max_diff ? diff : max_diff;
                    //std::cout << "diff: " << diff << ",  max_diff: " << max_diff << "\n";
                }
            }
            //std::cout << "Final max_diff: " << max_diff << "\n";

            // Binary Haptic Effort
            // Keep simple for now and make more complex after testing.
            if (max_diff > 10 * 2.1223 / 1024){
                haptic_effort = 1;
                //std::cout << "Haptic Effort: " << haptic_effort << "\n";
            }else{
                haptic_effort = 0;
            }
        }
        if(hand_lock) haptic_effort = 0;

        for (int i = 0; i < 4; i++) {
            hand_state[i] = msg.position[i];
            hand_state_set = true;
        }
    }
    catch (const std::exception& e){
        ROS_ERROR("Error with Haptic Effort: %s", e.what());
    }

    // Initialise spread pos at hand's current position.
    /*
    if( == 0){
        spread_pos = (hand_state[3] < SPREAD_CLOSE_TARGET/2) ? 1 : -1;
    }
     */
}

void handle_controller::send_handle_feedback() {
    ROS_DEBUG("Sending Handle Feedback");

    // Convert Haptic Effort Value to Byte
    double a;
    uint8_t effort;
    /*
    haptic_effort += 0.5; // Previously 1
    if(haptic_effort > 1){
        haptic_effort = 0;
    }
     */
    a = haptic_effort*(HAP_MAX - HAP_MIN) + HAP_MIN; // Scale effort between max and min
    effort = (uint8_t)round(a/100 * (255)); // convert to uint8_t
    //std::cout << "Effort: " << (int)effort << std::endl;

    // Set LED and Tx/Rx
    uint8_t data_bits = 0x0;
    data_bits = data_bits | TXRX_TRANSMIT;
    if(handle_led_set){
        data_bits = data_bits | LED_MASK;
    }
    //std::cout << a << std::endl;

    struct can_frame can_msg;
    can_msg.can_id = HANDLE_ID;
    can_msg.can_dlc = 8;
    can_msg.data[0] = 0x00;
    can_msg.data[1] = 0x00;
    can_msg.data[2] = 0x00;
    can_msg.data[3] = 0x00;
    can_msg.data[4] = effort;
    can_msg.data[5] = 0x00;
    can_msg.data[6] = 0x00;
    can_msg.data[7] = data_bits;


    for(int i=1; (i < 10) && ros::ok(); i++){
        send_port(&can_msg);
        if(read_port()) return;
        ROS_WARN("Failed to receive CAN message from handle. Retrying %i ...", i);
    }
    throw std::runtime_error("Reached maximum number of retries.");
}

/// Trigger Logic
/// Function: The proportional value from the trigger will be scaled from 0-2^10
///           to 0-2.1223rad. 0x0000 from the handle is fully open and 0x03ff is
///           fully closed.
void handle_controller::process_trigger(int trigger){
    if((trigger < 0) || (trigger > 1023)){
        ROS_ERROR("Received an invalid trigger value: %i", trigger);
    }

    //std_msgs::Float64 test_msg;
    //test_msg.data = (float)(FINGER_CLOSE_TARGET / 1023 * (double)trigger);
    //pb_raw.publish(test_msg);

    float value = filtered_trigger.get_median((float)(FINGER_CLOSE_TARGET / 1023 * (double)trigger));

    //test_msg.d//ata = a_test;
    //pb_fil.publish(test_msg);

    hand_tgt_pos[0] = value;
    hand_tgt_pos[1] = value;
    hand_tgt_pos[2] = value;

    // Even if hand is locked we should continue feeding values to the filter.
    wam_srvs::BHandGraspPos srv;
    //srv.request.radians = filtered_trigger.get_median((float)(2.1223 / 1023 * (double)trigger));
    srv.request.radians = value;
    if(!hand_lock) {
        if(!grasp_client.call(srv)){
            ROS_ERROR("Failed to call service: grasp_pos");
        }
    }
}


/// Joystick Logic
/// Function:
///           the joystick may be used to control a cursor on the host computer.
void handle_controller::process_joy(int joy1, int joy2){
    if((joy1 < 0) || (joy1 > 256)){
        ROS_ERROR("Received an invalid trigger value: %i", joy1);
    }
    float filt_joy1 = filtered_joy1.get_median(joy1);
    //float filt_joy2 = filtered_joy2.get_median((float)(SPREAD_CLOSE_TARGET / 255 * (double)joy2));

    float value = hand_tgt_pos[3];
    if(abs(filt_joy1-256/2) > 256/4 ){
        value = (float)(sgn(filt_joy1 - 256/2)*SPREAD_RATE/loop_rate) + hand_tgt_pos[3];
        value = (value > SPREAD_CLOSE_TARGET) ? SPREAD_CLOSE_TARGET : value;
        value = (value < SPREAD_OPEN_TARGET) ? SPREAD_OPEN_TARGET : value;
        hand_tgt_pos[3] = (float)value;
    }

    wam_srvs::BHandSpreadPos srv;
    srv.request.radians = value;

    if(!hand_lock) {
        if(!spread_client.call(srv)){
            ROS_ERROR("Failed to call service: spread_pos.");
        }
    }
}

/// Joystick Button Logic
/// Function: When the button is pressed it will toggle the spread of the bhand
///           between the 'W' (close) and 'Y' (open) positions based on the
///           bhand's current position.
/// True  - Button is depressed.
/// False - Button is released.
void handle_controller::process_button_joy(bool button_joy){
    // Trigger toggle on rising edge of button press action with "last_button_joy".
    if(button_joy && !last_button_joy && !hand_lock && timer_joy.isReady() && hand_state_set){
        timer_joy.set();
        std_srvs::Empty srv;

        if (hand_state[3] > SPREAD_CLOSE_TARGET/2){
            if(spread_open_client.call(srv)){
                hand_tgt_pos[3] = SPREAD_OPEN_TARGET;
            }
            else {
                ROS_ERROR("Failed to call service: open_spread");
            }
        }
        else if (hand_state[3] <= SPREAD_CLOSE_TARGET/2){
            if(spread_close_client.call(srv)){
                hand_tgt_pos[3] = SPREAD_CLOSE_TARGET;
            }
            else {
                ROS_ERROR("Failed to call service: close_spread");
            }
        }
    }
    last_button_joy = button_joy;
}

/// Push Button Logic
/// Function: When the button is pressed it will suppress bhand commands.
///           This 'locks' the bhand's position. The lock is only toggled
///           when the botton state goes from 0 to 1.
/// True  - Button is depressed.
/// False - Button is released.
void handle_controller::process_button_push(bool button_push){
    // Trigger toggle on rising edge of button press action.
    // TODO: Weird flashing bug here. When button is held LED changes.
    if(button_push && !last_button_push && timer_push.isReady()){
        ROS_INFO("Butt");
        timer_push.set();
        hand_lock = !hand_lock;
        handle_led_set = !handle_led_set;
    }
    last_button_push = button_push;
}

/// CAN PACKET STRUCTURE //
// TODO: add doc for packet structure once finalized
void handle_controller::unpack_reply(struct can_frame *packet){
    if (packet->can_id != HANDLE_ID){
        ROS_INFO("Received message with an unrecognized ID. Ignoring...");
        return;
    }
    //ROS_INFO("Received message from Handle.");

    handle_interface::handle_state handle_state;

    // Send raw values to Handle Node
    handle_state.trigger = (packet->data[0] << 8 | packet->data[1]) & 0x03FF;
    handle_state.joy1 = packet->data[2];
    handle_state.joy2 = packet->data[3];
    handle_state.button_joy = packet->data[7] & JOYSTICK_BUTTON_MASK;
    handle_state.button_push = packet->data[7] & PUSH_BUTTON_MASK;

    //Print Message
    /*
    std::cout << "CAN Message received.\n";
    for(int i=0; i<8; i++){
        std::cout << "Byte " << i << ": 0x" << std::hex << (int)packet->data[i] << std::endl;
    }
    *
    std::cout << "Trigger: " << (int) handle_state.trigger << std::endl;
    std::cout << "Joy1   : " << (int) handle_state.joy1 << std::endl;
    std::cout << "Joy2   : " << (int) handle_state.joy2 << std::endl;
    std::cout << "B_Joy  : " << (int) handle_state.button_joy << std::endl;
    std::cout << "B_Push : " << (int) handle_state.button_push << std::endl;

    /*j
    bool txrx = packet->data[7] & TXRX_MASK;
    std::cout << "TXRX   : " << (int) txrx << std::endl;
    if (txrx != TXRX_RECIEVE){
        //This should never happen but may be good for error correcting
        ROS_ERROR("Received a 'transmit' handle CAN message when a 'receive' message was expected.");
    }
     */
    hand_tgt_set = true;

    process_button_push((bool)handle_state.button_push);
    process_button_joy((bool)handle_state.button_joy);
    process_trigger(handle_state.trigger);
    process_joy(handle_state.joy1, handle_state.joy2);

    std_msgs::Float64MultiArray tgt_msg;
    tgt_msg.data.resize(4);
    std_msgs::Float64 hap_msg;

    if(hand_tgt_set){
        for(int i=0; i<4; i++){ tgt_msg.data[i] = hand_tgt_pos[i]; }
        hap_msg.data = haptic_effort;
    }

    handle_state_pub.publish(handle_state);
    hand_tgt_pub.publish(tgt_msg);
    handle_haptic_effort.publish(hap_msg);

}

bool handle_controller::start() {
    nh.param<double>("loop_rate", loop_rate, 100);
    ros::Rate rate(loop_rate);
    ROS_INFO("Node started");

    while(ros::ok()){
        send_handle_feedback();

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "handle_controller");
    try{
        handle_controller handle("can0");
        handle.start();
    }catch(const std::exception &e){
        std::cout<< "\033[1;31m[ERROR] An exception was caught, with message: ";
        std::cout<< e.what() << std::endl;
    }
    return 0;
}
