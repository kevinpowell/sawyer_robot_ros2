#include "sim_sawyer_robot_driver/sim_sawyer_robot_driver.hpp"
#include "intera_core_msgs/msg/joint_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <cmath>
#include <string>
#include <vector>
#include <rclcpp/executors.hpp>

#define START_IND 0
namespace sim_sawyer_robot_driver {

    CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo &info) {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        intera_core_msgs::msg::JointCommand cmd_msg;
        sensor_msgs::msg::JointState state_msg;


        // robot has 7 arm joints + head_pan and 2 interfaces
        joint_position_.assign(8, 0);
        joint_velocities_.assign(8, 0);
        joint_efforts_.assign(8, 0);
        joint_names_.assign(8, "");
        //joint_names_[0] = "head_pan";


        // offer position and velocity control
        joint_velocities_command_.assign(8, 0);
        joint_positions_command_.assign(8,0);
        last_jpc_.assign(8,0);
        delta_j_.assign(8,0);

        int ind = START_IND;
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

        int ind = START_IND;
        for (const auto &joint_name: joint_interfaces["velocity"]) {
            state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
        }
        ind = START_IND;
        for (const auto &joint_name: joint_interfaces["position"]) {
            state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
        }
        ind = START_IND;
        for (const auto &joint_name: joint_interfaces["effort"]) {
            state_interfaces.emplace_back(joint_name, "effort", &joint_efforts_[ind++]);
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        int ind = START_IND;
        for (const auto &joint_name: joint_interfaces["velocity"]) {
            command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
        }

        ind = START_IND;
        for (const auto &joint_name: joint_interfaces["position"]) {
            command_interfaces.emplace_back(joint_name, "position", &joint_positions_command_[ind++]);
        }

        return command_interfaces;
    }

    return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period) {
        for (auto i = 0ul; i < joint_velocities_command_.size(); i++) {
            joint_velocities_[i] = joint_velocities_command_[i];
            joint_position_[i] += joint_velocities_command_[i] * period.seconds();
        }

        if(std::any_of(delta_j_.begin(), delta_j_.end(), [](auto n){return n != 0.0;})) {
           for(int i=0; i < std::size(delta_j_); i++) {
             auto incr = std::copysign(period.seconds() * 1.0, delta_j_[i]);  //1 rad/sec constant joint vel in position mode
             if(incr > 0.0)
               incr = std::min(incr, delta_j_[i]);
             else
               incr = std::max(incr, delta_j_[i]);
             joint_position_[i] += incr;
             delta_j_[i] -= incr;
             if(incr > 0.0)
                delta_j_[i] = std::max(delta_j_[i], 0.0);
             else
                delta_j_[i] = std::min(delta_j_[i], 0.0);
           }
        }
        return return_type::OK;
    }

    return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &) {
        for (auto i = 0ul; i < joint_velocities_command_.size(); i++) {
            joint_velocities_[i] = joint_velocities_command_[i];
        }

        if(!std::equal(joint_positions_command_.begin(), joint_positions_command_.end(), last_jpc_.begin())) {
          std::copy(joint_positions_command_.begin(), joint_positions_command_.end(), last_jpc_.begin());
          for(int i=0; i < std::size(last_jpc_); i++)
             delta_j_[i] = last_jpc_[i] - joint_position_[i];
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
