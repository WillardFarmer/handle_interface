#ifndef SRC_DEBOUNCE_TIMER_H
#define SRC_DEBOUNCE_TIMER_H

#include <ros/ros.h>

class debounce_timer{
    public:
    explicit debounce_timer(const double& duration) : _is_set(false), _duration(duration) {}
    void set();
    bool isReady();

    private:
    bool _is_set;
    double _start_time;
    double _duration;
};

/// set() will record the current time and the desired wait duration.
/// \param duration
/// \return
void debounce_timer::set() {
    _is_set = true;
    _start_time = ros::Time::now().toSec();
}

/// is_ready() will return True when the set duration has passed or if
/// set() has not yet been called. It will return false otherwise.
/// \return
bool debounce_timer::isReady() {
    if(!_is_set) return true;
    double time_diff = ros::Time::now().toSec() - _start_time;
    return time_diff > _duration;
}
#endif //SRC_DEBOUNCE_TIMER_H
