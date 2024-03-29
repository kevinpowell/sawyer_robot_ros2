#include "sim_sawyer_robot_driver/sim_sawyer_robot_driver.hpp"
#include "intera_core_msgs/msg/joint_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <string>
#include <vector>
#include <rclcpp/executors.hpp>

namespace sim_sawyer_robot_driver {

    CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo &info) {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        intera_core_msgs::msg::JointCommand cmd_msg;
        sensor_msgs::msg::JointState state_msg;


        // robot has 7 joints and 2 interfaces
        joint_position_.assign(10, 0);
        joint_velocities_.assign(10, 0);
        joint_efforts_.assign(10, 0);
        joint_names_.assign(10, "");
        joint_names_[0] = "head_pan";
        joint_names_[1] = "right_gripper_r_finger_joint";
        joint_names_[2] = "right_gripper_r_finger_joint";


        // only offer velocity control
        joint_velocities_command_.assign(10, 0);

        int ind = 3;
        for (const auto &joint: info_.joints) {
            joint_names_[ind++] = joint.name;
            for (const auto &interface: joint.state_interfaces) {
                joint_interfaces[interface.name].push_back(joint.name);
            }
        }

        terminate_thread_ = false;
        cmd_node_ = std::make_shared<rclcpp::Node>("cmd_node");
        cmd_publisher_ = rclcpp::create_publisher<sensor_msgs::msg::JointState>(cmd_node_, "joint_states", 10);

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        int ind = 3;
        for (const auto &joint_name: joint_interfaces["velocity"]) {
            state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
        }
        ind = 3;
        for (const auto &joint_name: joint_interfaces["position"]) {
            state_interfaces.emplace_back(joint_name, "position", &joint_velocities_[ind++]);
        }
        ind = 3;
        for (const auto &joint_name: joint_interfaces["effort"]) {
            state_interfaces.emplace_back(joint_name, "effort", &joint_velocities_[ind++]);
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        int ind = 3;
        for (const auto &joint_name: joint_interfaces["velocity"]) {
            command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
        }

        return command_interfaces;
    }

    return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period) {
        for (auto i = 0ul; i < joint_velocities_command_.size(); i++) {
            joint_velocities_[i] = joint_velocities_command_[i];
            joint_position_[i] += joint_velocities_command_[i] * period.seconds();
        }

        return return_type::OK;
    }

    return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &) {
        for (auto i = 0ul; i < joint_velocities_command_.size(); i++) {
            joint_velocities_[i] = joint_velocities_command_[i];
        }
        
        sensor_msgs::msg::JointState msg;
        msg.name = joint_names_;
        msg.velocity = joint_velocities_;
        msg.position = joint_position_;
        msg.header.stamp = cmd_node_->now();

        cmd_publisher_->publish(msg);

        return return_type::OK;
    }

}  // namespace sawyer_robot_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(sim_sawyer_robot_driver::RobotSystem, hardware_interface::SystemInterface)
