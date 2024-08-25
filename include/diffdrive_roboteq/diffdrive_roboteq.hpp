#ifndef DIFFDRIVE_ROBOTEQ_HPP
#define DIFFDRIVE_ROBOTEQ_HPP

#include <mutex>
#include <vector>
#include <string>
#include <sstream>
#include <map>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <serial/serial.h>
#include "rclcpp/time.hpp"
#include "std_msgs/msg/string.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "diffdrive_roboteq/query_config.hpp"
#include "diffdrive_roboteq/visibility_control.h"


namespace diffdrive_roboteq
{
    class DiffDriveRoboteqHardware : public hardware_interface::SystemInterface
    {
        public:
            DIFFDRIVE_ROBOTEQ_PUBLIC
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

            DIFFDRIVE_ROBOTEQ_PUBLIC
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            DIFFDRIVE_ROBOTEQ_PUBLIC
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            DIFFDRIVE_ROBOTEQ_PUBLIC
            hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

            DIFFDRIVE_ROBOTEQ_PUBLIC
            hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

            DIFFDRIVE_ROBOTEQ_PUBLIC
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

            DIFFDRIVE_ROBOTEQ_PUBLIC
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

            DIFFDRIVE_ROBOTEQ_PUBLIC
            hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

            DIFFDRIVE_ROBOTEQ_PUBLIC
            hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

            DIFFDRIVE_ROBOTEQ_PUBLIC
            void extract_hardware_parameters(const hardware_interface::HardwareInfo & info);

            DIFFDRIVE_ROBOTEQ_PUBLIC
            void initialise_interfaces(const hardware_interface::HardwareInfo & info);

            DIFFDRIVE_ROBOTEQ_PUBLIC
            hardware_interface::CallbackReturn validate_joint_config(const hardware_interface::HardwareInfo & info);

            DIFFDRIVE_ROBOTEQ_PUBLIC
            void stop_motors();

            DIFFDRIVE_ROBOTEQ_PUBLIC
            void configure_hardware();

            DIFFDRIVE_ROBOTEQ_PUBLIC
            void load_and_write_queries();

            DIFFDRIVE_ROBOTEQ_PUBLIC
            void set_encoder_counts();

            DIFFDRIVE_ROBOTEQ_PUBLIC
            void clean_up_result_string(std_msgs::msg::String &result);

            DIFFDRIVE_ROBOTEQ_PUBLIC
            void process_result(std_msgs::msg::String &result);

            DIFFDRIVE_ROBOTEQ_PUBLIC
            void print_results(const std::map<std::string, std::string> &data_map);

            DIFFDRIVE_ROBOTEQ_PUBLIC
            void process_encoder_data(const std::map<std::string, std::string> &data_map);

            DIFFDRIVE_ROBOTEQ_PUBLIC
            void process_encoder_values(const std::string &encoder_speed_value, const std::string &encoder_count_value);

            DIFFDRIVE_ROBOTEQ_PUBLIC
            void update_hardware_interfaces(double left_linear_velocity, double right_linear_velocity, double left_enc_angle, double right_enc_angle, double left_encoder_speed, double right_encoder_speed);

            DIFFDRIVE_ROBOTEQ_PUBLIC
            void prepare_command_string(std::stringstream &cmd_str);

        private:
            struct Config
            {
                std::string serial_port = "";
                int baud_rate = 57600;
                bool closed_loop = false;
                double wheel_radius = 0;
                double wheel_circumference = 0;
                double max_rpm = 0;
                int frequency = 0;
                int count_per_revolution = 0;
                double gear_reduction = 0;
                std::string query_config = "";
            };

            Config                      conf_;
            serial::Serial              ser_;
            QueryConfig                 query_config_;

            std::vector<double>         hw_commands_;
            std::vector<double>         hw_positions_;
            std::vector<double>         hw_velocities_;
            std::vector<double>         hw_encoder_speed_;
            std::vector<std::string>    topic_list_;

            std::mutex                  locker;
    };

} // namespace diffdrive_roboteq

#endif // DIFFDRIVE_ROBOTEQ_HPP