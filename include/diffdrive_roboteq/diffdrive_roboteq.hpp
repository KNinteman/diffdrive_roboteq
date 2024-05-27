#ifndef DIFFDRIVE_ROBOTEQ_HPP
#define DIFFDRIVE_ROBOTEQ_HPP

#include "hardware_interface/system_interface.hpp"
#include "diffdrive_roboteq/query_config.hpp"
#include "diffdrive_roboteq/visibility_control.h"

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <mutex>
#include <serial/serial.h>
#include "rclcpp/time.hpp"
#include "std_msgs/msg/string.hpp"


namespace diffdrive_roboteq
{
    class DiffDriveRoboteqHardware : public hardware_interface::SystemInterface
    {
        struct Config 
        {
            std::string serial_port = "";
            int baud_rate = 57600;
            bool closed_loop = true;
            double wheel_radius = 0;
            double wheel_circumference = 0;
            double max_rpm = 0;
            int frequency = 0;
            int count_per_revolution = 0;
            double gear_reduction = 0;
            std::string query_config = "";
        };
        
        public:
            DIFFDRIVE_ROBOTEQ_PUBLIC
            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo & info) override;

            DIFFDRIVE_ROBOTEQ_PUBLIC
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            DIFFDRIVE_ROBOTEQ_PUBLIC
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            DIFFDRIVE_ROBOTEQ_PUBLIC
            hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State & previous_state) override;

            DIFFDRIVE_ROBOTEQ_PUBLIC
            hardware_interface::CallbackReturn on_cleanup(
                const rclcpp_lifecycle::State & previous_state) override;

            DIFFDRIVE_ROBOTEQ_PUBLIC
            hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State & previous_state) override;

            DIFFDRIVE_ROBOTEQ_PUBLIC
            hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State & previous_state) override;

            DIFFDRIVE_ROBOTEQ_PUBLIC
            hardware_interface::return_type read(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;

            DIFFDRIVE_ROBOTEQ_PUBLIC
            hardware_interface::return_type write(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;

            DIFFDRIVE_ROBOTEQ_PUBLIC
                void stopMotors();

            DIFFDRIVE_ROBOTEQ_PUBLIC
                void configureHardware();
            
            DIFFDRIVE_ROBOTEQ_PUBLIC
                void loadAndWriteQueries();

            // DIFFDRIVE_ROBOTEQ_PUBLIC
            //     void setEncoderCounts();

            DIFFDRIVE_ROBOTEQ_PUBLIC
                void cleanUpResultString(std_msgs::msg::String& result);

            DIFFDRIVE_ROBOTEQ_PUBLIC
                void processResult(std_msgs::msg::String& result);

            DIFFDRIVE_ROBOTEQ_PUBLIC
                void printResults(const std::map<std::string, std::string>& data_map);

            DIFFDRIVE_ROBOTEQ_PUBLIC
                void processEncoderData(const std::map<std::string, std::string>& data_map);
            
            DIFFDRIVE_ROBOTEQ_PUBLIC
                void processEncoderValues(const std::string& encoder_speed_value, const std::string& encoder_count_value);
            
            DIFFDRIVE_ROBOTEQ_PUBLIC
                void updateHardwareInterfaces(double left_linear_velocity, double right_linear_velocity, double left_enc_angle, double right_enc_angle);
            
            DIFFDRIVE_ROBOTEQ_PUBLIC
                void prepareCommandString(std::stringstream& cmd_str);
            
            DIFFDRIVE_ROBOTEQ_PUBLIC
                void prepareClosedLoopCommand(std::stringstream& cmd_str, float left_speed, float right_speed);
            
            DIFFDRIVE_ROBOTEQ_PUBLIC
                void prepareOpenLoopCommand(std::stringstream& cmd_str, float left_speed, float right_speed);

        private:
            Config                  conf_;
            serial::Serial 			ser_;
            QueryConfig             query_config_;

            std::vector<double>     hw_commands_;
            std::vector<double>     hw_positions_;
            std::vector<double>     hw_velocities_;
            std::vector<std::string>    topic_list_;

            std::mutex 				locker;

    };

} // namespace diffdrive_roboteq

#endif // DIFFDRIVE_ROBOTEQ_HPP