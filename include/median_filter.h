#ifndef SRC_MEDIAN_FILTER_H
#define SRC_MEDIAN_FILTER_H

#include <bits/stdc++.h>

/**
 * Moving median filter used to filter incoming sensor data.
 * May wish to filter joystick and trigger values so a class
 * was created. _window value specifies the window size for
 * the filter.
 */
class median_filter{
public:
    median_filter(const int w) : _window(w) {}
    float get_median(float);

private:
    const int _window;
    std::deque<float> _raw_values;

    float calc_median(std::deque<float>);
};

/**
 * Method to get filtered value. Class stores previous n-1 values
 * (where n is the window length). It should be noted that the output
 * value is delayed by (n-1)/2 values, since the filter requires
 * 'future' values to determine a median. Filters where the
 * output value is the median of the last n measurements were found
 * to be less accurate than using this delayed method. The first (n-1)/2
 * will not be valid but given the high frequency, this weakness is
 * acceptable.
 * TODO: Make generic beyond float
 * @param value the value to be filtered
 * @return filtered value
 */
float median_filter::get_median(float value) {
    _raw_values.push_front(value);
    if (_raw_values.size() > _window) _raw_values.resize(_window); // Keep to window size

    return calc_median(_raw_values);
}

// Code modified from https://www.geeksforgeeks.org/median-of-stream-of-running-integers-using-stl/
float median_filter::calc_median(std::deque<float> raw_values){

    // max heap to store the smaller half elements
    std::priority_queue<float> s;
    // min heap to store the greater half elements
    std::priority_queue<float,std::vector<float>,std::greater<float>> g;

    float med = raw_values.front();
    s.push(med);

    // reading elements of stream one by one
    /*  At any time we try to make heaps balanced and
        their sizes differ by at-most 1. If heaps are
        balanced,then we declare median as average of
        min_heap_right.top() and max_heap_left.top()
        If heaps are unbalanced,then median is defined
        as the top element of heap of larger size  */
    for (auto it = (raw_values.cbegin()+1); it != raw_values.cend(); ++it){
        //for(int i=0; i<raw_values.size(); i++){
        float x = *it;
        //float x = raw_values[i];

        // case1(left side heap has more elements)
        if (s.size() > g.size())
        {
            if (x < med)
            {
                g.push(s.top());
                s.pop();
                s.push(x);
            }
            else
                g.push(x);

            med = (s.top() + g.top())/2.0;
        }

        // case2(both heaps are balanced)
        else if (s.size()==g.size())
        {
            if (x < med)
            {
                s.push(x);
                med = (float)s.top();
            }
            else
            {
                g.push(x);
                med = (float)g.top();
            }
        }

        // case3(right side heap has more elements)
        else
        {
            if (x > med)
            {
                s.push(g.top());
                g.pop();
                g.push(x);
            }
            else
                s.push(x);

            med = (s.top() + g.top())/2.0;
        }
    }
    return med;
}

#endif
