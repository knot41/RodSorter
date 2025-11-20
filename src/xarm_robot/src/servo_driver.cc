#include "xarm_robot/servo_driver.h"
#include <functional>
#include <thread>

ServoMotor::ServoMotor(std::string port) {
    if (serial_driver.Open(port, SerialConfig())) {
        RCLCPP_INFO(rclcpp::get_logger("ServoMotor"), "Serial port opened successfully: %s", port.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("ServoMotor"), "Failed to open serial port: %s", port.c_str());
        serial_driver.Close();
        if (!serial_driver.Open("/dev/ttyACM1", SerialConfig())) {
            RCLCPP_ERROR(rclcpp::get_logger("ServoMotor"), "Failed to open serial port: %s", "/dev/ttyACM1");
        }
    }
    Enable();
    serial_driver.Write("config limit.speed=70\n");
    std::vector<uint8_t> output;
    serial_driver.ReadBytes(output, 100, 10);
    core_thread_ = std::thread(std::bind(&ServoMotor::core_thread, this));
    SetAngle(0);
    current_angle = 0;
}

void ServoMotor::Enable() {
    serial_driver.Write("enable\n");
    std::vector<uint8_t> output;
    serial_driver.ReadBytes(output, 100, 10);
}

void ServoMotor::SetAngle(float angle, int time) {
    this->thing_to_do = 0;
    this->angle = angle;
    this->time = time;
    this->flag = true;
}
void ServoMotor::Forward(float angle, int time) {
    this->thing_to_do = 1;
    this->angle = angle;
    this->time = time;
    this->flag = true;
}
void ServoMotor::Backward(float angle, int time) {
    this->thing_to_do = 2;
    this->angle = angle;
    this->time = time;
    this->flag = true;
}

void ServoMotor::SetAngle_P(float angle, int time) {
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
    angle > 2 * M_PI ? angle -= 2 * M_PI : angle;
    angle < 0 ? angle += 2 * M_PI : angle;

    current_angle = angle;
    serial_driver.Write("ctrl angle " + std::to_string(current_angle) + "\n");

    std::vector<uint8_t> output;
    serial_driver.ReadBytes(output, 100, 10);
    serial_driver.ReadBytes(output, 100, 10);
    if (output.size() == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ServoMotor"), "Failed to set angle: %f", angle);
        SetAngle(current_angle, 0);
    }
}

void ServoMotor::Forward_P(float angle, int time) {
    float target = current_angle + angle;

    SetAngle_P(target, time);
}

void ServoMotor::Backward_P(float angle, int time) {
    float target = current_angle - angle;
    SetAngle_P(target, time);
}

void ServoMotor::core_thread() {
    while (1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (flag == true) {
            if (thing_to_do == 0) {
                SetAngle_P(this->angle, this->time);
                flag = false;
            } else if (thing_to_do == 1) {
                Forward_P(this->angle, this->time);
                flag = false;
            } else if (thing_to_do == 2) {
                Backward_P(this->angle, this->time);
                flag = false;
            }
        }
    }
}