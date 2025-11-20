#include "chrono"
#include "rclcpp/rclcpp.hpp"
#include "serial_driver.h"
#include "thread"
#include <atomic>
#include <math.h>
#include <string>
#ifndef SERVO_DRIVER_H
    #define SERVO_DRIVER_H

class ServoMotor {
public:
    ServoMotor(std::string port = "/dev/ttyACM0");
    void SetAngle(float angle, int time = 0);
    void Forward(float angle, int time = 0);
    void Backward(float angle, int time = 0);
    void SetAngle_P(float angle, int time = 0);
    void Forward_P(float angle, int time = 0);
    void Backward_P(float angle, int time = 0);
    void Enable();
    void core_thread();
    float current_angle = 0;

private:
    SerialDriver serial_driver;
    std::thread core_thread_;
    std::atomic<bool> flag = false;
    std::atomic<int> thing_to_do = 0;
    std::atomic<float> angle=0;
    std::atomic<int> time=0;
};

#endif
