#ifndef PROJECT_STOPWATCH_H
#define PROJECT_STOPWATCH_H
#include <ros/ros.h>

class Stopwatch{
protected:
public:
    Stopwatch();
    void restart();
    ros::Duration elapsed();
private:
    ros::Time start_;
};
#endif