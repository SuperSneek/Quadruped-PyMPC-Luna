#include <message_converter/converter_base.hpp>
#include <cmath>
#include <algorithm>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dls2_msgs/msg/blind_state_msg.hpp>

namespace message_converter
{
	using JointState = sensor_msgs::msg::JointState;
  using BlindData = dls2_msgs::msg::BlindStateMsg;
  class BlindStateConverter : public message_converter::PluginBase
  {

    public:

        BlindStateConverter() = default;

        ~BlindStateConverter() override 
        {
          RCLCPP_INFO(node_->get_logger(), "Blind State Converter shutting down.");
        }

        std::string getName() override
        {
          return std::string("Blind State Converter");
        }
        std::string getDescription() override
        {
          return std::string("A converter that aggregates odometry, joint states and legodometry to produce state estimation");
        }

        void jointStateCallback(const JointState::ConstSharedPtr &joint_state)
        {
            // Build index mapping on first message
            if (index_mapping_.empty())
            {
                buildIndexMapping(joint_state);
            }
            
            //Set float timestamp
            msg_.timestamp = joint_state->header.stamp.sec + joint_state->header.stamp.nanosec * 1e-9;
            //set frame ID
            msg_.frame_id = joint_state->header.frame_id;

            // Reorder joint data according to joints_order_ parameter
            // Use the precomputed index mapping for efficiency
            for (size_t i = 0; i < index_mapping_.size() && i < msg_.joints_name.size(); ++i)
            {
                int src_idx = index_mapping_[i];
                if (src_idx >= 0 && src_idx < static_cast<int>(joint_state->name.size()))
                {
                    msg_.joints_name[i] = joint_state->name[src_idx];
                    
                    if (src_idx < static_cast<int>(joint_state->position.size()))
                        msg_.joints_position[i] = joint_state->position[src_idx];
                    
                    if (src_idx < static_cast<int>(joint_state->velocity.size()))
                        msg_.joints_velocity[i] = joint_state->velocity[src_idx];
                    
                    if (src_idx < static_cast<int>(joint_state->effort.size()))
                        msg_.joints_effort[i] = joint_state->effort[src_idx];
                }
            }
            
            //TODO: Calculate acceleration? Set to 0 for now
            msg_.joints_acceleration.fill(0.0);
            
            //TODO: Only use temperature if necessary
            msg_.joints_temperature.fill(0.0);
            
            // Assume all feet are in contact
            msg_.feet_contact = {1.0, 1.0, 1.0, 1.0};

            blind_state_pub_->publish(msg_);
        }

    protected:
        void initialize_() override
        {
            auto node = this->node_;
            RCLCPP_INFO(node_->get_logger(), "Blind State Converter initialized.");
            // TODO: Add subscriptions, publishers, timers here
            // declare parameters with sensible defaults and read their values
            auto joint_states_topic = node_->declare_parameter<std::string>("joint_states_topic", "/joint_states");
            auto pub_topic = node_->declare_parameter<std::string>("blind_pub_topic", "/blind_state_topic");
            joints_order_ = node_->declare_parameter<std::vector<std::string>>("joints_order", 
              {"FL_shoulder_joint", "FL_thigh_joint", "FL_calf_joint",
               "FR_shoulder_joint", "FR_thigh_joint", "FR_calf_joint",
               "RL_shoulder_joint", "RL_thigh_joint", "RL_calf_joint",
               "RR_shoulder_joint", "RR_thigh_joint", "RR_calf_joint"});

            //Subscriber
            auto qos = rclcpp::SensorDataQoS();
            joint_state_sub_ = node_->create_subscription<JointState>(
                joint_states_topic, qos,
                std::bind(&BlindStateConverter::jointStateCallback, this, std::placeholders::_1));

            //Publisher
            blind_state_pub_ = node_->create_publisher<BlindData>(pub_topic, qos);
        }

        void shutdown_() override
        {
            RCLCPP_INFO(node_->get_logger(), "Blind State Converter shutdown.");
        }

		void pause_() override
        {
            RCLCPP_INFO(node_->get_logger(), "Blind State Converter paused.");
        }

		void resume_() override
        {
            RCLCPP_INFO(node_->get_logger(), "Blind State Converter resumed.");
        }

		void reset_() override
        {
            RCLCPP_INFO(node_->get_logger(), "Blind State Converter reset.");
        }

    private:
        rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
        rclcpp::Publisher<BlindData>::SharedPtr blind_state_pub_;
        BlindData msg_;
        
        std::vector<std::string> joints_order_;
        std::vector<int> index_mapping_;  // Precomputed mapping from desired order to incoming order
        
        void buildIndexMapping(const JointState::ConstSharedPtr &first_joint_state)
        {
            index_mapping_.resize(joints_order_.size(), -1);
            
            for (size_t i = 0; i < joints_order_.size(); ++i)
            {
                // Find the index of joints_order_[i] in the incoming joint_state
                auto it = std::find(first_joint_state->name.begin(), 
                                   first_joint_state->name.end(), 
                                   joints_order_[i]);
                
                if (it != first_joint_state->name.end())
                {
                    index_mapping_[i] = std::distance(first_joint_state->name.begin(), it);
                }
                else
                {
                    RCLCPP_WARN(node_->get_logger(), 
                               "Joint '%s' from joints_order not found in incoming joint_states", 
                               joints_order_[i].c_str());
                }
            }
        }

  };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(message_converter::BlindStateConverter, message_converter::PluginBase)