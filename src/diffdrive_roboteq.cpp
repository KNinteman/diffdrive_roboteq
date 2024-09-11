#include "diffdrive_roboteq/diffdrive_roboteq.hpp"

namespace diffdrive_roboteq
{ 
    /**
    * @brief Initialises the hardware with the provided configuration and checks joint configuration.
    *
    * This method intialises the hardware with the configuration provided in the hardware info paramters 
      and initialises the vectors that hold the for the command and state interfaces.
    * Additionally, it validates whether the amount of joint are the same as the amount of joints that are expected.
    *
    * @param info The hardware info object containing configuration parameters and joint information.
    * @return hardware_interface::CallbackReturn The initialization status, indicating whether the
    * initialisation was successful or if an error occurred.

    */
    hardware_interface::CallbackReturn DiffDriveRoboteqHardware::on_init(const hardware_interface::HardwareInfo & info)
        {
            // Check if class initialisation is successful
            if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
                return hardware_interface::CallbackReturn::ERROR;
            }

            extract_hardware_parameters(info);
            initialise_interfaces(info);

            validate_joint_config(info);

            return hardware_interface::CallbackReturn::SUCCESS;
        }

    /**
    * @brief Extracts hardware parameters from the provided hardware info parameters.
    *
    * This method extracts parameters from the hardware info parameter's yaml file and initialises internal variables accordingly. 
    * These parameters are crucial for configuring and operating the hardware.
    *
    * @param info The hardware info object containing configuration parameters and joint information.
    */
    void DiffDriveRoboteqHardware::extract_hardware_parameters(const hardware_interface::HardwareInfo & info)
    {
        conf_.serial_port = info_.hardware_parameters["serial_port"];
        conf_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
        conf_.closed_loop = (info_.hardware_parameters["closed_loop"] == "true");
        conf_.wheel_radius = hardware_interface::stod(info_.hardware_parameters["wheel_radius"]);
        conf_.wheel_circumference = hardware_interface::stod(info_.hardware_parameters["wheel_circumference"]);
        conf_.max_rpm = hardware_interface::stod(info_.hardware_parameters["max_rpm"]);
        conf_.frequency = std::stoi(info_.hardware_parameters["frequency"]);
        conf_.count_per_revolution = std::stoi(info_.hardware_parameters["count_per_revolution"]);
        conf_.gear_reduction = hardware_interface::stod(info_.hardware_parameters["gear_reduction"]);   
        conf_.query_config = info_.hardware_parameters["query_config"];  
    }

    /**
    * @brief initialises the vectors for command and state interfaces.
    *
    * This method initialises the hardware interfaces for positions, velocities, commands and encoder speeds
    * It resizes the internal vectors to accommodate the number of joints.
    * Each vector is initialised with default values (0.0)
    *
    * @param info The hardware info object containing configuration parameters and joint information.
    */
    void DiffDriveRoboteqHardware::initialise_interfaces(const hardware_interface::HardwareInfo & info)
    {
        hw_positions_.resize(info_.joints.size(), 0.0);
        hw_velocities_.resize(info_.joints.size(), 0.0);
        hw_commands_.resize(info_.joints.size(), 0.0);
        hw_encoder_speed_.resize(info_.joints.size(), 0.0);
    }

    /**
    * @brief Validates the configuration of joints in the hardware.
    *
    * This method validates the configuration of each joint. 
    * It makes sure that each joint has exactly one command interface of type
    * velocity and exactly two state interfaces of types position and velocity.
    *
    * @param info The hardware info object containing configuration parameters and joint information.
    * @return hardware_interface::CallbackReturn The validation status, indicating whether the
    * joint configuration is correct or if an error occurred.
    */
    hardware_interface::CallbackReturn DiffDriveRoboteqHardware::validate_joint_config(const hardware_interface::HardwareInfo & info)
    {
        for (const auto & joint : info_.joints)
            {
                // Check that each joint has exactly one command interface
                if (joint.command_interfaces.size() != 1)
                {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("DiffDriveRoboteqHardware"),
                        "Joint '%s' has %zu command interfaces found. 1 expected.", 
                        joint.name.c_str(), joint.command_interfaces.size());
                    return hardware_interface::CallbackReturn::ERROR;
                }

                // Check that the command interface is of type velocity
                if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
                {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("DiffDriveRoboteqHardware"),
                        "Joint '%s' have %s command interfaces found. '%s' expected.", 
                        joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                    return hardware_interface::CallbackReturn::ERROR;
                }

                // Check that each joint has exactly two state interfaces
                if (joint.state_interfaces.size() != 2)
                {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("DiffDriveRoboteqHardware"),
                        "Joint '%s' has %zu state interface. 2 expected.", 
                        joint.name.c_str(), joint.state_interfaces.size());
                    return hardware_interface::CallbackReturn::ERROR;
                }

                // Check that the first state interface is of type position
                if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
                {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("DiffDriveRoboteqHardware"),
                        "Joint '%s' have '%s' as first state interface. '%s' expected.", 
                        joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                    return hardware_interface::CallbackReturn::ERROR;
                }

                // Check that the second state interface is of type velocity
                if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
                {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("DiffDriveRoboteqHardware"),
                        "Joint '%s' have '%s' as second state interface. '%s' expected.", 
                        joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                    return hardware_interface::CallbackReturn::ERROR;
                }
            }
    }

    /**
    * @brief Exports the state interfaces provided by the hardware.
    *
    * This method defines and exports the state interfaces for each joint. Each joint
    * is associated with position and velocity state interfaces. These interfaces allow
    * the hardware to export its states (position and velocity).
    * 
    * @return std::vector<hardware_interface::StateInterface> A vector containing the state interfaces for all joints.
    */
    std::vector<hardware_interface::StateInterface> DiffDriveRoboteqHardware::export_state_interfaces()
    {
        // Create a vector to hold the state interfaces
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // Reserve space in the vector
        constexpr std::size_t state_interfaces_per_joint = 2;
        state_interfaces.reserve(info_.joints.size() * state_interfaces_per_joint);

        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            // Add position state interface for the current joint
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            
            // Add velocity state interface for the current joint
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));

            // Add encoder_speed for the current joint
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, "encoder_speed", &hw_positions_[i]));
        }

        return state_interfaces;
    }
    

    /**
    * @brief Exports the command interfaces provided by the hardware.
    *
    * This method defines and exports the command interfaces for each joint. Each joint
    * is associated with a velocity command interface. These interfaces allow the controller
    * to export velocity commands.
    * 
    * @return std::vector<hardware_interface::CommandInterface> A vector containing the command interfaces for all joints.
    */
    std::vector<hardware_interface::CommandInterface> DiffDriveRoboteqHardware::export_command_interfaces()
    {
        // Create a vector to hold the command interfaces
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            // Add velocity command interface for the current joint
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
        }

        return command_interfaces;
    }

    /**
    * @brief Starts hardware communication with the Roboteq Motor Controller.
    * 
    * This method sets up the communication to the hardware and prepares it for activation.
    * It initialises the serial port with the specified settings such as baud rate.
    * 
    * @param previous_state The previous lifecycle state of the hardware component.
    * @return CallbackReturn Returns a status indicating the success or failure of the configuration process.
    */
    hardware_interface::CallbackReturn DiffDriveRoboteqHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {   
        try {
            ser_.setPort(conf_.serial_port);
            ser_.setBaudrate(conf_.baud_rate);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(timeout);
            ser_.open();
        } catch (serial::IOException &e) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("DiffDriveRoboteqHardware"), "Serial Port " << conf_.serial_port << " is not open.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (ser_.isOpen()) {
            // If the port is open, log a message indicating successful initialization
            RCLCPP_INFO_STREAM(rclcpp::get_logger("DiffDriveRoboteqHardware"), "Serial Port " << conf_.serial_port << " initialised.");
        } else {
            // If the port is not open, log a message indicating failure and shutdown the ROS node
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("DiffDriveRoboteqHardware"), "Serial Port " << conf_.serial_port << " is not open.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
    * @brief Cleans up the hardware communication.
    * 
    * This method closes the serial port.
    * 
    * @param previous_state The previous lifecycle state of the hardware component.
    * @return CallbackReturn Returns a status indicating the success or failure of the cleanup process.
    */
    hardware_interface::CallbackReturn DiffDriveRoboteqHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("DiffDriveRoboteqHardware"), "Cleaning up ...please wait...");
        
        try {
            if (ser_.isOpen()) {
                ser_.close();
                RCLCPP_INFO_STREAM(rclcpp::get_logger("DiffDriveRoboteqHardware"), "Serial Port " << conf_.serial_port << " closed.");
            } else {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("DiffDriveRoboteqHardware"), "Serial Port " << conf_.serial_port << " was not open.");
            }
        } catch (const serial::IOException &e) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("DiffDriveRoboteqHardware"), "Error while closing serial port: " << e.what());
        }

        RCLCPP_INFO_STREAM(rclcpp::get_logger("DiffDriveRoboteqHardware"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
    * @brief Stops the motors.
    * 
    * Sends commands to the serial port interface (ser_) to stop motor 1 and motor 2.
    * After sending the commands, it flushes the serial port buffer to make sure its sends immediately.
    */
    void DiffDriveRoboteqHardware::stop_motors() {
        ser_.write("!G 1 0\r");
        ser_.write("!G 2 0\r");
        ser_.write("!S 1 0\r");
        ser_.write("!S 2 0\r");
        ser_.flush();
    }

    /**
    * @brief Configures the motors.
    * 
    * Enables the watchdog timer and sets the motor operating mode based on configuration parameters.
    * If closed loop control is disabled, sets motor operating mode to open loop (0),
    * otherwise sets it to closed loop (1) for both motors.
    */
    void DiffDriveRoboteqHardware::configure_hardware() {
        // Enable watchdog timer
        ser_.write("^RWD 1000\r");

        // Set motor operating mode
        if (!conf_.closed_loop) {
            ser_.write("^MMOD 1 0\r");
            ser_.write("^MMOD 2 0\r");
        } else {
            ser_.write("^MMOD 1 1\r");
            ser_.write("^MMOD 2 1\r");
        }
        ser_.flush();
    }

    /**
    * @brief Sets the encoder counts for the motors.
    * 
    * Creates commands to set the encoder counts for motor 1 and motor 2, 
    * then sends these commands to the serial port interface (ser_).
    * After sending the commands, it flushes the serial port buffer to make sure its sends immediately.
    */
    void DiffDriveRoboteqHardware::set_encoder_counts() {
        std::stringstream right_enccmd;
        std::stringstream left_enccmd;
        right_enccmd << "^EPPR 1 " << 1024 << "\r";
        left_enccmd << "^EPPR 2 " << 1024 << "\r";
        ser_.write(right_enccmd.str());
        ser_.write(left_enccmd.str());
        ser_.flush();
    }

    /**
    * @brief Loads query configurations and writes them to the serial port.
    * 
    * Loads query configurations from the provided configuration (conf_.query_config),
    * formats the queries into two stringstreams (ss0 and ss1), and writes them to the serial port interface (ser_).
    * After sending the commands, it flushes the serial port buffer to make sure its sends immediately.
    */
    void DiffDriveRoboteqHardware::load_and_write_queries() {
        query_config_ = load_query_config(conf_.query_config);

        std::stringstream ss0, ss1;
        ss0 << "^echof 1_";
        ss1 << "# c_/\"DH?\",\"?\"";        
    
        // Iterate over the queries and format them into the stringstream
        for (const auto& query : query_config_.queries) {
            std::string topic = query.first;
            std::string query_str = query.second;
            
            ss1 << query_str << "_"; 
            topic_list_.push_back(topic);
        }

        ss1 << "# " << conf_.frequency << "_";

        // Write commands to serial
        ser_.write(ss0.str());
        ser_.write(ss1.str());
        ser_.flush();
    }

    /**
    * @brief Activates the hardware components.
    * 
    * Checks if the serial port is open.
    * If it is it stops the motors, configures hardware settings, loads and writes queries to the serial port,
    * and returns a success status upon successful activation. 
    */
    hardware_interface::CallbackReturn DiffDriveRoboteqHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardware"), "Activing ...please wait...");
        // Check if the serial port is open
        if (!ser_.isOpen()) {
            RCLCPP_ERROR(
                rclcpp::get_logger("DiffDriveRoboteqHardware"), 
                "Serial port is not open. Unable to activate.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        try {
            stop_motors();
            configure_hardware();
            // set_encoder_counts();
            load_and_write_queries();
        } catch (const serial::IOException &e) {
            RCLCPP_ERROR_STREAM(
                rclcpp::get_logger("DiffDriveRoboteqHardware"), 
                "Error while configuring hardware: " << e.what()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief Deactivates the hardware components.
     * 
     * Currently not implemented.
     * 
     */
    hardware_interface::CallbackReturn DiffDriveRoboteqHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("DiffDriveRoboteqHardware"), "Deactivating ...please wait...");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("DiffDriveRoboteqHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
    * @brief Generates the current date and time with milliseconds.
    * 
    * Retrieves the current system time and formats it as a string in the format "YYYY-MM-DD HH:MM:SS.sss",
    * where "sss" represents milliseconds.
    * 
    * @return The current date and time with milliseconds, as a string.
    */
    std::string current_date_time() {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        
        auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - seconds).count();
        
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time), "%F %T.") << std::setw(3) << std::setfill('0') << ms;
        
        return ss.str();
    }

    /**
    * @brief Cleans up the result string by removing specific characters.
    *
    * This function removes return ("\r"), plus ("+"), and "DH?" substrings
    * from the given result string. It modifies the input string in-place.
    *
    * @param result The string to be cleaned up.
    */
    void DiffDriveRoboteqHardware::clean_up_result_string(std_msgs::msg::String& result) {
        boost::replace_all(result.data, "\r", "");
        boost::replace_all(result.data, "+", ""); 
        boost::replace_all(result.data, "DH?", ""); 
    }

    /**
    * @brief Processes the result string into a map of data.
    *
    * This function splits the given result string based on the "?" delimiter
    * and stores the resulting substrings into a map, using topic names from
    * the `topic_list_` as keys. If the number of substrings matches the number
    * of topics, it prints the results to the console and processes encoder data.
    *
    * @param result The result string to be processed.
    */
    void DiffDriveRoboteqHardware::process_result(std_msgs::msg::String& result) {
        if (!result.data.empty()) {
            std::vector<std::string> results;
            boost::split(results, result.data, boost::is_any_of("?"));
            if (results.size() != topic_list_.size()) {
                RCLCPP_WARN(
                    rclcpp::get_logger("DiffDriveRoboteqHardware"),
                    "Unexpected number of results received: %zu, expected: %zu",
                    results.size(), topic_list_.size()
                );
                return;
            } else {
                // Store the results in a map
                std::map<std::string, std::string> data_map;
                for (size_t i = 0; i < results.size(); ++i) {
                    data_map[topic_list_[i]] = results[i];
                }
                print_results(data_map);
                process_encoder_data(data_map);
            }
        }
    }

    /**
    * @brief Logs the contents of the data map to the console.
    *
    * This function logs the current timestamp along with the contents of the
    * given data map to the console. It iterates over each key-value pair in the
    * map and logs them in the format "key: value".
    *
    * @param data_map The map containing data to be printed.
    */
    void DiffDriveRoboteqHardware::print_results(const std::map<std::string, std::string>& data_map) {
        // Print the current time
        RCLCPP_INFO_STREAM(
            rclcpp::get_logger("DiffDriveRoboteqHardware"),
            "timestamp: " << current_date_time()
        );

        // Print the map to the console
        for (const auto& pair : data_map) {
            RCLCPP_INFO_STREAM(
                rclcpp::get_logger("DiffDriveRoboteqHardware"),
                pair.first << ": " << pair.second
            );
        }
    }

    /**
    * @brief Processes encoder data from the given data map.
    *
    * This function checks if the given data map contains entries for encoder speed
    * and encoder count topics. If both topics are present, it extracts their values
    * from the map and passes them to the process_encoder_values function for further
    * processing.
    *
    * @param data_map The map containing encoder data.
    */
    void DiffDriveRoboteqHardware::process_encoder_data(const std::map<std::string, std::string>& data_map)
    {
        const std::string encoder_speed_topic = "encoder_speed";
        const std::string encoder_count_topic = "encoder_count";
        
        if (data_map.find(encoder_speed_topic) != data_map.end() &&
            data_map.find(encoder_count_topic) != data_map.end()) {
            
            std::string encoder_speed_value = data_map.at(encoder_speed_topic);
            std::string encoder_count_value = data_map.at(encoder_count_topic);

            process_encoder_values(encoder_speed_value, encoder_count_value);
        }
    }

    /**
    * @brief Processes encoder speed and count values.
    *
    * This function splits the given encoder speed and count values and calculates
    * the corresponding linear velocities and encoder angles for left and right wheels.
    * It updates the hardware interfaces with the calculated values if the data format
    * is valid.
    *
    * @param encoder_speed_value The string containing encoder speed values.
    * @param encoder_count_value The string containing encoder count values.
    */
    void DiffDriveRoboteqHardware::process_encoder_values(const std::string& encoder_speed_value, const std::string& encoder_count_value) {
        // Split the values for left and right wheels
        std::vector<std::string> speed_values;
        std::vector<std::string> count_values;
        boost::split(speed_values, encoder_speed_value, boost::is_any_of(":"));
        boost::split(count_values, encoder_count_value, boost::is_any_of(":"));

        if (speed_values.size() == 2 && count_values.size() == 2) {
            int left_encoder_speed = std::stoi(speed_values[0]);  
            int right_encoder_speed = std::stoi(speed_values[1]); 
            int left_encoder_count = std::stoi(count_values[0]);
            int right_encoder_count = std::stoi(count_values[1]);

            double left_rpm = static_cast<double>(left_encoder_speed);
            double right_rpm = static_cast<double>(right_encoder_speed);
            double left_linear_velocity = ((left_rpm / conf_.gear_reduction) / 60.0) * (2 * M_PI * conf_.wheel_radius);
            double right_linear_velocity = ((right_rpm / conf_.gear_reduction) / 60.0) * (2 * M_PI * conf_.wheel_radius);
            double rads_per_count = (2 * M_PI) / (conf_.count_per_revolution * conf_.gear_reduction);
            double left_enc_angle = left_encoder_count * rads_per_count;
            double right_enc_angle = right_encoder_count * rads_per_count;

            update_hardware_interfaces(left_linear_velocity, right_linear_velocity, left_enc_angle, right_enc_angle, left_encoder_speed, right_encoder_speed);
        } else {
            std::cout << "Invalid data format for encoder values." << std::endl;
        }
    }

    /**
    * @brief Updates the hardware interfaces with new values.
    *
    * This function updates the hardware interface arrays with the given left and right
    * linear velocities and encoder angles. It also prints the updated values to the console
    * for verification purposes.
    *
    * @param left_linear_velocity The linear velocity of the left wheel.
    * @param right_linear_velocity The linear velocity of the right wheel.
    * @param left_enc_angle The encoder angle of the left wheel.
    * @param right_enc_angle The encoder angle of the right wheel.
    */
    void DiffDriveRoboteqHardware::update_hardware_interfaces(double left_linear_velocity, double right_linear_velocity, 
        double left_enc_angle, double right_enc_angle, double left_encoder_speed, double right_encoder_speed)
    {
        hw_velocities_[0] = left_linear_velocity;
        hw_velocities_[1] = right_linear_velocity;
        hw_positions_[0] = left_enc_angle;
        hw_positions_[1] = right_enc_angle;
        hw_encoder_speed_[0] = left_encoder_speed;
        hw_encoder_speed_[1] = right_encoder_speed;

        // Log the updated values separately for clarity
        RCLCPP_INFO_STREAM(
            rclcpp::get_logger("DiffDriveRoboteqHardware"),
            "linear_velocity (L): " << left_linear_velocity
        );
        RCLCPP_INFO_STREAM(
            rclcpp::get_logger("DiffDriveRoboteqHardware"),
            "linear_velocity (R): " << right_linear_velocity
        );
        RCLCPP_INFO_STREAM(
            rclcpp::get_logger("DiffDriveRoboteqHardware"),
            "encoder_angle (L): " << left_enc_angle
        );
        RCLCPP_INFO_STREAM(
            rclcpp::get_logger("DiffDriveRoboteqHardware"),
            "encoder_angle (R): " << right_enc_angle
        );
        RCLCPP_INFO_STREAM(
            rclcpp::get_logger("DiffDriveRoboteqHardware"),
            "encoder_speed (L): " << left_encoder_speed
        );
        RCLCPP_INFO_STREAM(
            rclcpp::get_logger("DiffDriveRoboteqHardware"),
            "encoder_speed (R): " << right_encoder_speed
        );
        RCLCPP_INFO_STREAM(
            rclcpp::get_logger("DiffDriveRoboteqHardware"),
            "-----------------------------"
        );
    }

    /**
    * @brief Reads data from the serial port and processes it.
    *
    * This function reads data from the serial port and processes it if the port is open.
    * It first checks if the serial port is open, returning an error if it's not. If data
    * is available, it reads the data, cleans it up, and processes it. Any exceptions during
    * the process are caught and logged. Returns OK if successful, otherwise returns ERROR.
    *
    * @param time The current time (unused).
    * @param period The duration of the current time period (unused).
    * @return hardware_interface::return_type The status of the read operation.
    */
    hardware_interface::return_type DiffDriveRoboteqHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    {
        try {
            if (!ser_.isOpen()) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("DiffDriveRoboteqHardware"), 
                    "Serial port is not open. Unable to read."
                );
                return hardware_interface::return_type::ERROR;
            }
            
            if (ser_.available()) {
                std_msgs::msg::String result;
                result.data = ser_.read(ser_.available());
                clean_up_result_string(result);
                process_result(result);
            }
            return hardware_interface::return_type::OK;
        } catch (const std::exception &e) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("DiffDriveRoboteqHardware"), "Exception in read(): " << e.what());
            return hardware_interface::return_type::ERROR;
        }
    } 
    
    /**
    * @brief Prepares the command string based on current commands and configuration.
    *
    * This function prepares the command string for controlling the motors based on the
    * current commands and configuration settings. It determines whether to use closed-loop
    * or open-loop commands and delegates the preparation accordingly.
    *
    * @param cmd_str The stringstream to store the prepared command string.
    */
    void DiffDriveRoboteqHardware::prepare_command_string(std::stringstream& cmd_str)
    {
        float left_speed  = hw_commands_[0];
        float right_speed = hw_commands_[1];

        float right_power = right_speed * 100; //285 MAX. SET TO 100 IN ORDER TO INFLUENCE MAX SPEED.
        float left_power  = left_speed  * 100; //285 MAX

        // if (left_power < 0) {
        //     right_power -= left_power;
        //     left_power = 0;
        // };

        cmd_str << "!G 1" << " " << (int)left_power << "_"
                << "!G 2" << " " << (int)right_power << "_";
    }

    /**
    * @brief Writes commands to the hardware.
    * 
    * This method directly sends the commands to the motors.
    * 
    * @param time The current time (unused).
    * @param period The time elapsed since the last write operation (unused).
    * @return hardware_interface::return_type Returns a status indicating the success or failure of the write operation.
    */
    hardware_interface::return_type diffdrive_roboteq ::DiffDriveRoboteqHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        try {
            // Check if the serial port is open
            if (!ser_.isOpen()) {
                RCLCPP_ERROR(rclcpp::get_logger("DiffDriveRoboteqHardware"), "Serial port is not open. Unable to activate.");
                return hardware_interface::return_type::ERROR;
            }
            
            std::stringstream cmd_str;
            prepare_command_string(cmd_str);

            ser_.write(cmd_str.str());
            ser_.flushInput();

            return hardware_interface::return_type::OK;
        } catch (const std::exception &e) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("DiffDriveRoboteqHardware"), "Exception in write(): " << e.what());
            return hardware_interface::return_type::ERROR;
        }
    }

} // namespace diffdrive_roboteq
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_roboteq::DiffDriveRoboteqHardware, hardware_interface::SystemInterface)