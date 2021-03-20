#ifndef SRC_HANDLE_CONTROLLER_H
#define SRC_HANDLE_CONTROLLER_H

#include <exception>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <libsocketcan.h>

#include <ros/ros.h>
#include <handle_interface/handle_state.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "wam_srvs/BHandGraspPos.h"
#include "wam_srvs/BHandSpreadPos.h"
#include "std_srvs/Empty.h"

#include "debounce_timer.h"

// Handle constants
#define HANDLE_ID 0x102
#define JOYSTICK_BUTTON_MASK 0x08
#define PUSH_BUTTON_MASK 0x04
#define LED_MASK 0x02
#define TXRX_MASK 0x01
#define TXRX_TRANSMIT 0
#define TXRX_RECIEVE 1
#define HAP_MAX 50
#define HAP_MIN 0

#define FINGER_OPEN_TARGET 0         // [rad]
#define SPREAD_OPEN_TARGET 0         // [rad]
#define FINGER_CLOSE_TARGET 2.1223   // [rad]
#define SPREAD_CLOSE_TARGET 3.1416   // [rad]

class handle_controller {

    public:
    handle_controller(const char*);
    ~handle_controller();

    bool start(int);

    private:
    ros::NodeHandle nh;

    ros::Publisher handle_state_pub;
    ros::Subscriber hand_state_sub;

    ros::ServiceClient grasp_client;
    ros::ServiceClient spread_open_client;
    ros::ServiceClient spread_close_client;

    // CAN Methods
    void open_port(const char*);   //TODO: change to voids when exception handling has been added.
    void send_port(struct can_frame*);
    bool read_port();
    void close_port();
    void unpack_reply(struct can_frame*);

    // Handle Methods
    void send_handle_feedback();
    void hand_pos_callback(const sensor_msgs::JointState&);

    void process_trigger(int);
    void process_joy(int, int);
    void process_button_joy(bool);
    void process_button_push(bool);

    int soc;

    bool hand_tgt_set;
    bool handle_led_set;
    double hand_state[4];
    double hand_tgt_pos[4];
    double haptic_effort;    // Between 0 and 1

    bool hand_lock;
    int spread_pos;         // 1 (open), 0 (unset), -1 (closed)
    bool last_button_joy;
    bool last_button_push;
    debounce_timer timer_joy;
    debounce_timer timer_push;

    uint8_t test;
};

#endif //SRC_HANDLE_CONTROLLER_H
