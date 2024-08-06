// std lib
#include <chrono>
#include <memory>
#include <vector>
#include <math.h>
#include <string>

// ros2
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

// cr_hardware
#include "cr_hardware/differential_drive.hpp"

hardware_interface::CallbackReturn DifferentialDrive::on_init(
    const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    node_handle_ = std::make_shared<rclcpp::Node>("parameter_reader_node");
    
    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    debug_ = info_.hardware_parameters["debug"].compare("True") == 0 ? true : false;
    RCLCPP_INFO(
        rclcpp::get_logger("DifferentialDrive"),
        "DEBUG MODE: %s", debug_ ? "True" : "False");
    RCLCPP_INFO(
        rclcpp::get_logger("DifferentialDrive"),
        "DEBUG String: %s", info_.hardware_parameters["debug"].c_str());
    serial_port_ = info_.hardware_parameters["serial_port"];
    RCLCPP_INFO(
        rclcpp::get_logger("DifferentialDrive"),
        "Serial port: %s", serial_port_.c_str());
    baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
    RCLCPP_INFO(
        rclcpp::get_logger("DifferentialDrive"),
        "Baud rate: %d", baud_rate_);
    wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
    RCLCPP_INFO(
        rclcpp::get_logger("DifferentialDrive"),
        "Wheel Radius: %f", wheel_radius_);
    circumference_ = 2 * wheel_radius_ * M_PI;

    // verify and error check joint types and interfaces
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("DifferentialDrive"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("DifferentialDrive"),
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("DifferentialDrive"),
                "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("DifferentialDrive"),
                "Joint '%s' have %s state interfaces found. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("DifferentialDrive"),
                "Joint '%s' have %s state interfaces found. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("DifferentialDrive"), "Successfully initialized differential drive.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DifferentialDrive::on_configure(
    const rclcpp_lifecycle::State &previous_state
)
{
    rc_ = roboclaw_init(serial_port_.c_str(), baud_rate_);
    if (rc_ == NULL)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("DifferentialDrive"),
            "Failed to configure Roboclaw.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    else
    {
        RCLCPP_INFO(
            rclcpp::get_logger("DifferentialDrive"),
            "Differential Drive has been configured.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }
}

hardware_interface::CallbackReturn DifferentialDrive::on_cleanup(
    const rclcpp_lifecycle::State &previous_state)
{
    // cleanup memory
    RCLCPP_INFO(
        rclcpp::get_logger("DifferentialDrive"),
        "Differential Drive has been cleaned up.");
    
    if (rc_ != nullptr)
        roboclaw_close(rc_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DifferentialDrive::on_deactivate(
    const rclcpp_lifecycle::State &previous_state)
{
    // stop motor controllers
    RCLCPP_INFO(
        rclcpp::get_logger("DifferentialDrive"),
        "Differential Drive has been deactivated.");
    roboclaw_duty_m1m2(rc_, 128, 0, 0);
    roboclaw_duty_m1m2(rc_, 129, 0, 0);
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DifferentialDrive::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DifferentialDrive::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn DifferentialDrive::on_activate(
    const rclcpp_lifecycle::State &previous_state)
{
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
        hw_positions_[i] = 0.0;
        hw_velocities_[i] = 0.0;
        hw_commands_[i] = 0.0;
    }

    // roboclaw_connections_ = std::make_shared<libroboclaw::driver>();

    RCLCPP_INFO(
        rclcpp::get_logger("DifferentialDrive"),
        "Differential Drive has been activated.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DifferentialDrive::read(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    // Read the state from the robot hardware
    // hw_positions_[0] = ...
    // hw_positions_[1] = ...
    // hw_velocities_[0] = ...
    // hw_velocities_[1] = ...
    if (rc_ == nullptr)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("DifferentialDrive"),
            "Roboclaw is not initialized.");
        return hardware_interface::return_type::ERROR;
    }  


    int32_t enc_fl, enc_bl, enc_fr, enc_br;
    int speed_fl, speed_bl, speed_fr, speed_br;
    int res = 0;
    res = roboclaw_encoders(rc_, 128, &enc_fl, &enc_bl);
    if (res == ROBOCLAW_OK)
    {
        // convert encoders to meters, (2*pi*r) * (revolutions)
        double revolutions = (double)(enc_fl * revolutions_per_enc_tick_);
        hw_positions_[0] = revolutions * circumference_;
    }
    // else
    // {
    //     RCLCPP_ERROR(
    //         rclcpp::get_logger("DifferentialDrive"),
    //         "Failed to read encoder values from Roboclaw.");
    //     return hardware_interface::return_type::ERROR;
    // }

    res = roboclaw_encoders(rc_, 129, &enc_fr, &enc_br);
    if (res == ROBOCLAW_OK)
    {
        // convert encoders to meters, (2*pi*r) * (revolutions)
        double revolutions = (double)(enc_fr * revolutions_per_enc_tick_);
        hw_positions_[1] = revolutions * circumference_;
    }
    // else
    // {
    //     RCLCPP_ERROR(
    //         rclcpp::get_logger("DifferentialDrive"),
    //         "Failed to read encoder values from Roboclaw.");
    //     return hardware_interface::return_type::ERROR;
    // }

    res = 0;
    res = roboclaw_read_speed_m1(rc_, 128, &speed_fl);
    if (res == ROBOCLAW_OK)
    {
        // convert encoder speed to meters per second
        double revolutions_per_sec = (double)(speed_fl * revolutions_per_enc_tick_);
        hw_velocities_[0] = revolutions_per_sec * circumference_;
    }
    // else
    // {
    //     RCLCPP_ERROR(
    //         rclcpp::get_logger("DifferentialDrive"),
    //         "Failed to read encoder values from Roboclaw.");
    //     return hardware_interface::return_type::ERROR;
    // }

    res = 0;
    res = roboclaw_read_speed_m1(rc_, 129, &speed_fr);
    if (res == ROBOCLAW_OK)
    {
        // convert encoder speed to meters per second
        double revolutions_per_sec = (double)(speed_fr * revolutions_per_enc_tick_);
        hw_velocities_[1] = revolutions_per_sec * circumference_;
    }
    // else
    // {
    //     RCLCPP_ERROR(
    //         rclcpp::get_logger("DifferentialDrive"),
    //         "Failed to read encoder values from Roboclaw.");
    //     return hardware_interface::return_type::ERROR;
    // }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DifferentialDrive::write(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    // Send the command to the robot hardware

    // revolutions per sec [rev/s] * circumference [m] gives meters per sec [m/s]
    double revolutions_per_meter = 1 / circumference_;
    int speed_left_cmd = hw_commands_[0] * wheel_radius_ * revolutions_per_meter * enc_ticks_per_revolution_;
    int speed_right_cmd = hw_commands_[1] * wheel_radius_ * revolutions_per_meter * enc_ticks_per_revolution_;
    int res;
    
    // left wheels, 6600 accel means reach full speed in one second. Max speed is 6600 ticks per second
    res = roboclaw_speed_accel_m1m2(rc_, 128, speed_left_cmd, speed_left_cmd, 6600 * 2);
    if (res != ROBOCLAW_OK)
    {
        return hardware_interface::return_type::ERROR;
    }

    // right wheels
    res = 0;
    res = roboclaw_speed_accel_m1m2(rc_, 129, speed_right_cmd, speed_right_cmd, 6600 * 2);
    if (res != ROBOCLAW_OK)
    {
        return hardware_interface::return_type::ERROR;
    }

    if (hw_commands_[0] != 0 && debug_)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("DifferentialDrive"),
            "Sending velocity command to left wheel: %f", hw_commands_[0]);
    }
    if (hw_commands_[1] != 0 && debug_)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("DifferentialDrive"),
            "Sending velocity command to right wheel: %f", hw_commands_[1]);
    }
    return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(DifferentialDrive, hardware_interface::SystemInterface)