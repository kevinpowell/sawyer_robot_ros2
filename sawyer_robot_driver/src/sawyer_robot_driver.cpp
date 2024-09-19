#include "sawyer_robot_driver/sawyer_robot_driver.hpp"
#include "intera_core_msgs/msg/joint_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <string>
#include <vector>
#include <rclcpp/executors.hpp>

namespace sawyer_robot_driver {
    using std::placeholders::_1;


    MinimalSubscriber::MinimalSubscriber(std::unordered_map<std::string, std::vector<std::string>> joint_interfaces)
            : Node("minimal_subscriber") {
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("robot/joint_states", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

        int ind = 0;
        for (const auto &joint: joint_interfaces["position"]) {
            ind_map_[joint] = ind;
            ind++;
        }
        joint_position_.assign(7, 0);
        joint_velocities_.assign(7, 0);
        joint_efforts_.assign(7, 0);


    }

    void MinimalSubscriber::topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        mutex_.lock();
        for (auto i = 0; i < msg->name.size(); i++) {
            std::string name(msg->name[i]);
            if (ind_map_.find(name) != ind_map_.end()) {
                int ind = ind_map_[name];
                joint_position_[ind] = msg->position[i];
                joint_velocities_[ind] = msg->velocity[i];
                joint_efforts_[ind] = msg->effort[i];
            }
        }
        mutex_.unlock();
    }

    void MinimalSubscriber::update_state(std::vector<double> &joint_position, std::vector<double> &joint_velocities,
                                         std::vector<double> &joint_efforts) {
        mutex_.lock();
        joint_position = joint_position_;
        joint_velocities = joint_velocities_;
        joint_efforts = joint_efforts_;
        mutex_.unlock();
    }


    CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo &info) {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }


        intera_core_msgs::msg::JointCommand cmd_msg;
        sensor_msgs::msg::JointState state_msg;


        // robot has 7 joints and 2 interfaces
        joint_position_.assign(7, 0);
        joint_velocities_.assign(7, 0);
        joint_efforts_.assign(7, 0);
        joint_names_.assign(7, "");

        // offer position and velocity control
        joint_velocities_command_.assign(7, 0);
        joint_positions_command_.assign(7,0);
        last_jpc_.assign(7,0);

        int ind = 0;
        for (const auto &joint: info_.joints) {
            joint_names_[ind++] = joint.name;
            for (const auto &interface: joint.state_interfaces) {
                joint_interfaces[interface.name].push_back(joint.name);
            }
        }

        terminate_thread_ = false;
        state_reader_node_ = std::make_shared<MinimalSubscriber>(joint_interfaces);
        cmd_node_ = std::make_shared<rclcpp::Node>("cmd_node");
        cmd_publisher_ = rclcpp::create_publisher<intera_core_msgs::msg::JointCommand>(cmd_node_, "/robot/limb/right/joint_command", 10);
        state_reader_thread_ = std::make_shared<std::thread>([this]() {
            bool terminate = terminate_thread_;
            while (!terminate_thread_) {
                rclcpp::spin(state_reader_node_);
                mutex.lock();
                terminate = terminate_thread_;
                mutex.unlock();
            }
        });


        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        int ind = 0;
        for (const auto &joint_name: joint_interfaces["velocity"]) {
            state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
        }
        ind = 0;
        for (const auto &joint_name: joint_interfaces["position"]) {
            state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
        }
        ind = 0;
        for (const auto &joint_name: joint_interfaces["effort"]) {
            state_interfaces.emplace_back(joint_name, "effort", &joint_efforts_[ind++]);
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        int ind = 0;
        for (const auto &joint_name: joint_interfaces["velocity"]) {
            command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
        }

        ind = 0;
        for (const auto &joint_name: joint_interfaces["position"]) {
            command_interfaces.emplace_back(joint_name, "position", &joint_positions_command_[ind++]);
        }
        return command_interfaces;
    }

    return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period) {
        state_reader_node_->update_state(joint_position_, joint_velocities_, joint_efforts_);

        return return_type::OK;
    }

    return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &) {
        //for (auto i = 0ul; i < joint_velocities_command_.size(); i++) {
        //    joint_velocities_[i] = joint_velocities_command_[i];
        //}

        intera_core_msgs::msg::JointCommand cmd_msg;
        bool send = true;
        cmd_msg.names = joint_names_;
        cmd_msg.header.stamp = cmd_node_->now();
        if(std::any_of(joint_velocities_command_.begin(), joint_velocities_command_.end(), [](auto n){return n != 0;})) {
          cmd_msg.mode = intera_core_msgs::msg::JointCommand::VELOCITY_MODE;
          cmd_msg.velocity = joint_velocities_command_;
        } else if (!std::equal(joint_positions_command_.begin(), joint_positions_command_.end(), last_jpc_.begin())) {
          cmd_msg.mode = intera_core_msgs::msg::JointCommand::POSITION_MODE;
          cmd_msg.position = joint_positions_command_;
          last_jpc_ = joint_positions_command_;
        } else {
          send = false;
        }

        if(send) cmd_publisher_->publish(cmd_msg);

        return return_type::OK;
    }

}  // namespace sawyer_robot_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(sawyer_robot_driver::RobotSystem, hardware_interface::SystemInterface)
