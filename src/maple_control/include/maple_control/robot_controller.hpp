#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class RobotController : public rclcpp::Node
{
public:
    RobotController();
    ~RobotController() override;

private:
    struct MotorConfig
    {
        int id = -1;
        int profile_acceleration = 0;
        int profile_speed = 0;
        std::vector<double> goal_positions;
    };

    struct MotionConfig
    {
        std::vector<MotorConfig> motors;
        std::vector<double> pause_durations;
    };

    enum class SleepResult
    {
        Done,
        Restart,
        Stop
    };

    void on_motion_command(const std_msgs::msg::String::SharedPtr msg);
    void on_interaction_control(const std_msgs::msg::String::SharedPtr msg);
    void run_motion(const std::string &motion_name);

    bool wait_until_running();
    SleepResult interruptible_sleep(double seconds);

    bool initialize_robot();
    void shutdown_robot();

    bool load_motion_config(const std::string &motion_name, MotionConfig &config);
    static int motor_angle_to_value(double angle);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr interaction_sub_;

    std::atomic<bool> run_event_{true};
    std::atomic<bool> stop_event_{false};
    std::atomic<bool> motion_in_progress_{false};

    std::mutex state_mutex_;
    std::string active_motion_;
    int active_pose_index_ = -1;

    std::thread motion_thread_;

    dynamixel::PortHandler *port_handler_ = nullptr;
    dynamixel::PacketHandler *packet_handler_ = nullptr;
    dynamixel::GroupSyncWrite *group_sync_write_ = nullptr;
    dynamixel::GroupSyncRead *group_sync_read_ = nullptr;

    std::string device_name_;
    std::string motion_dir_;
    int baudrate_ = 1000000;
    double protocol_version_ = 2.0;
    int motor_count_ = 12;
    int first_motor_id_ = 0;

    int addr_torque_enable_ = 64;
    int addr_goal_position_ = 116;
    int len_goal_position_ = 4;
    int addr_present_position_ = 132;
    int len_present_position_ = 4;
    int addr_profile_velocity_ = 112;
    int addr_profile_acceleration_ = 108;

    int torque_enable_ = 1;
    int torque_disable_ = 0;
};