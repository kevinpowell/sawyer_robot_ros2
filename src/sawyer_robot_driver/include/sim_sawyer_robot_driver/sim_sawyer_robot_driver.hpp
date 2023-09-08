#pragma once

#include <thread>
#include <mutex>
#include <rclcpp/node.hpp>
#include "string"
#include "unordered_map"
#include "vector"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "intera_core_msgs/msg/joint_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using hardware_interface::return_type;

namespace sim_sawyer_robot_driver {
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


    class HARDWARE_INTERFACE_PUBLIC RobotSystem : public hardware_interface::SystemInterface {
    public:

        ~RobotSystem() {
            mutex.lock();
            terminate_thread_ = false;
            mutex.unlock();
            state_reader_thread_->join();
        }

        CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

    protected:
        /// The size of this vector is (standard_interfaces_.size() x nr_joints)
        std::vector<double> joint_velocities_command_;
        std::vector<double> joint_position_;
        std::vector<double> joint_velocities_;
        std::vector<double> joint_efforts_;
        std::vector<std::string> joint_names_;
        std::vector<double> ft_states_;
        std::vector<double> ft_command_;

        std::shared_ptr<rclcpp::Node> cmd_node_;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> cmd_publisher_;

        std::shared_ptr<std::thread> state_reader_thread_;
        bool terminate_thread_;
        std::mutex mutex;

        std::unordered_map<std::string, std::vector<std::string>> joint_interfaces;
    };

}  // namespace sim_sawyer_robot_driver
