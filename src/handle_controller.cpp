#include <handle_controller.h>

handle_controller::handle_controller(const char *port) {

    struct can_frame ex;
    ex.can_id = 0x102;
    ex.can_dlc = 8;
    ex.data[0] = 0x00;
    ex.data[1] = 0x00;
    ex.data[2] = 0x00;
    ex.data[3] = 0x00;
    ex.data[4] = 0x00;
    ex.data[5] = 0x00;
    ex.data[6] = 0x00;
    ex.data[7] = 0x00;
    //unpack_reply(&ex);

    this->open_port(port);

    hand_state_sub = nh.subscribe("joint_states", 1,
                                  &handle_controller::hand_pos_callback, this);

    grasp_client = nh.serviceClient<wam_srvs::BHandGraspPos>("grasp_pos");
    spread_open_client = nh.serviceClient<std_srvs::Empty>("open_spread");
    spread_close_client = nh.serviceClient<std_srvs::Empty>("close_spread");

    handle_led_set = false;

    hand_lock = false;
    spread_pos = 0;  //spread has not been set
    last_button_joy = false;
    last_button_push = false;

    led_test = 0;


    //send_port(&ex);
    //read_port();
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

int handle_controller::open_port(const char *port) {

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
    return 0;
}

int handle_controller::send_port(struct can_frame *frame)
{
    int retval;
    retval = write(soc, frame, sizeof(struct can_frame));
    if (retval != sizeof(struct can_frame))
    {
        //std::cout << soc << ", " << retval << std::endl;
        //std::cout << errno << std::endl;
        throw std::runtime_error("Could not write to port.");
        return (-1);
    }
    else
    {
        //ROS_INFO("Message successfully sent.");
        return (0);
    }
}

int handle_controller::read_port() {
    //ROS_INFO("Reading Port");
    struct can_frame frame_rd;
    int recvbytes = 0;

    int i = 0;
    int read_can_port = 1;
    //while(read_can_port && (i < 4)){
    do{
        i += 1;
        if (i > 1) ROS_INFO("Retry, %i ...", i); // todo make pretty
        if (i > 5) throw std::runtime_error("Reached maximum number of retries.");
        struct timeval timeout = {1, 0};
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(soc, &readSet);

        if (select((soc + 1), &readSet, NULL, NULL, &timeout) >= 0) {
//            if (!read_can_port)
//            {
//                break;
//            }
            if (FD_ISSET(soc, &readSet)) {
                recvbytes = read(soc, &frame_rd, sizeof(struct can_frame));
                if (recvbytes > 0) {
                    //ROS_INFO("Message Found");
                    unpack_reply(&frame_rd);
                    read_can_port = 0;
                }
            }
        }
    } while(read_can_port);
}

int handle_controller::close_port() {
    close(soc);
    return 0;
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
    //ROS_INFO("Received Barrett Hand Pos message.");
    /*
    std::cout << "Cur_Sta: ";
    std::cout << hand_state[0] << ", ";
    std::cout << hand_state[1] << ", ";
    std::cout << hand_state[2] << ", ";
    std::cout << hand_state[3] << "\n";
    std::cout << "Message: ";
    std::cout << msg.position[0] << ", ";
    std::cout << msg.position[1] << ", ";
    std::cout << msg.position[2] << ", ";
    std::cout << msg.position[3] << "\n";
     */

    try {
        // Very basic haptic logic. Will need further development.
        if (hand_tgt_set) {
            double diff = 0;

            for (int i = 0; i < 3; i++) {
                // If finger has not moved. 31*100 * 2.1223/17000 * 1/10 = 0.0387 rad/pub
                // This number is the angle a finger should have moved since the last pos message.
                // * 0.1 is a scaling factor to ensure the finger is stopped or very slow.
                if( abs(hand_state[i] - msg.position[i]) < (0.0387 * 0.1) ) {
                    double a = 0;
                    if (hand_tgt_pos[i] < msg.position[i]) continue;
                    a = (hand_tgt_pos[i] - msg.position[i]);
                    diff = a > diff ? a : diff;
                    //std::cout << "a: "<< a << ",  diff: " << diff << "\n";
                }
            }
            //std::cout << "Final diff: " << diff << "\n";

            // Binary Haptic Effort
            // Keep simple for now and make more complex after testing.
            if (diff > 0){
                haptic_effort = 1;
                //std::cout << "Haptic Effort: " << haptic_effort << "\n";
            }
        }

        for (int i = 0; i < 4; i++) {
            hand_state[i] = msg.position[i];
        }
    }
    catch (const std::exception& e){
        ROS_ERROR("Error with Haptic Effort: %s", e.what());
    }

    // Initialise spread pos at hand's current position.
    if(spread_pos == 0){
        spread_pos = (hand_state[3] < SPREAD_CLOSE_TARGET/2) ? 1 : -1;
    }
}

void handle_controller::send_handle_feedback() {

    //ROS_INFO("Sending Handle Feedback");

    // Convert Haptic Effort Value to Byte
    double a;
    uint8_t effort;
    haptic_effort += 1;
    if(haptic_effort > 1){
        haptic_effort = 0;
    }
    a = haptic_effort*(HAP_MAX - HAP_MIN) + HAP_MIN; // Scale effort between max and min

    //std::cout << unsigned(a) << std::endl;

    effort = (uint8_t)round(a/100 * (255)); // convert to uint8_t
    //std::cout << unsigned(effort) << std::endl;

    // Set LED and Tx/Rx
    uint8_t data_bits = 0x0;
    data_bits = data_bits | TXRX_TRANSMIT;
    //if(handle_led_set || led_test){
    if(handle_led_set){
        data_bits = data_bits | LED_MASK;
    }
    //led_test = !led_test;

    //std::cout<< "EFFORT: " << unsigned(effort) << ", " << unsigned(data_bits) << std::endl;

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

    send_port(&can_msg);
    read_port();
}

/// Trigger Logic
/// Function: The proportional value from the trigger will be scaled from 0-2^10
///           to 0-2.1223rad. 0x0000 from the handle is fully open and 0x03ff is
///           fully closed.
void handle_controller::process_trigger(int trigger){
    if(!hand_lock) {
        if((trigger < 0) || (trigger > 1023)){
            ROS_ERROR("Received an invalid trigger value: %i", trigger);
        }
        wam_srvs::BHandGraspPos srv;
        srv.request.radians = 2.1223 / 1023 * (float)trigger;
        if(!grasp_client.call(srv)){
            ROS_ERROR("Failed to call service: grasp_pos");
        }
    }
}


/// Joystick Logic
/// Function: Currently there is no functionality for the joystick. In the future,
///           the joystick may be used to control a cursor on the host computer.
void handle_controller::process_joy(int joy1, int joy2){
}

/// Joystick Button Logic
/// Function: When the button is pressed it will toggle the spread of the bhand
///           between the 'W' (close) and 'Y' (open) positions based on the
///           bhand's current position.
/// True  - Button is depressed.
/// False - Button is released.
void handle_controller::process_button_joy(bool button_joy){
    // Trigger toggle on rising edge of button press action.
    if(button_joy && !last_button_joy && !hand_lock){
        std_srvs::Empty srv;

        if (spread_pos < 0){
            if(spread_open_client.call(srv)){
                spread_pos *= -1;
                hand_tgt_pos[3] = SPREAD_OPEN_TARGET;
            }
            else {
                ROS_ERROR("Failed to call service: open_spread");
            }
        }
        else if (spread_pos > 0){
            if(spread_close_client.call(srv)){
                spread_pos *= -1;
                hand_tgt_pos[3] = SPREAD_CLOSE_TARGET;
            }
            else {
                ROS_ERROR("Failed to call service: close_spread");
            }
        }
        else{
            ROS_INFO("No position message has been recieved. Push button will be ignored.");
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
    if(button_push && !last_button_push){
        hand_lock = !hand_lock;
        handle_led_set = !handle_led_set;
    }
    last_button_joy = button_push;
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
    */
    std::cout << "Trigger: " << (int) handle_state.trigger << std::endl;
    std::cout << "Joy1   : " << (int) handle_state.joy1 << std::endl;
    std::cout << "Joy2   : " << (int) handle_state.joy2 << std::endl;
    std::cout << "B_Joy  : " << (int) handle_state.button_joy << std::endl;
    std::cout << "B_Push : " << (int) handle_state.button_push << std::endl;

    /*
    bool txrx = packet->data[7] & TXRX_MASK;
    std::cout << "TXRX   : " << (int) txrx << std::endl;
    if (txrx != TXRX_RECIEVE){
        //This should never happen but may be good for error correcting
        ROS_ERROR("Received a 'transmit' handle CAN message when a 'receive' message was expected.");
    }
    ROS_INFO("Processing stuff.....");
    */

    process_button_push((bool)handle_state.button_push);
    process_trigger(handle_state.trigger);
    process_joy(handle_state.joy1, handle_state.joy2);
    process_button_joy((bool)handle_state.button_joy);

}

bool handle_controller::start(int rate) {
    ros::Rate loop_rate(rate);
    ROS_INFO("Node started");

    /*
    std_srvs::Empty srv1;
    wam_srvs::BHandGraspPos srv2;
    spread_open_client.call(srv1);
    spread_close_client.call(srv1);
    srv2.request.radians = 2.1223 / 1023 * 0x3ff;
    grasp_client.call(srv2);
     */

    while(ros::ok()){
        send_handle_feedback();

        ros::spinOnce();
        loop_rate.sleep();
    }

}

int main(int argc, char **argv){
    ros::init(argc, argv, "handle_controller");
    try{
        handle_controller handle("can0");
        handle.start(10);
    }catch(const std::exception &e){
        std::cout<< "\033[1;31m[ERROR] An exception was caught, with message: ";
        std::cout<< e.what() << std::endl;
    }
    return 0;
}
