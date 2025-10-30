#include <message_converter/converter_base.hpp>
#include <cmath>
#include <sensor_msgs/msg/joint_state.hpp>
#include <state_estimator_msgs/msg/contact_detection.hpp>

#include <message_filters/time_synchronizer.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>



namespace message_converter
{
	using JointState = sensor_msgs::msg::JointState;
  using Contact = state_estimator_msgs::msg::ContactDetection;
  using ApproximateTimePolicy = message_filters::sync_policies::ApproximateTime<JointState, Contact>;
	using ExactTimePolicy = message_filters::sync_policies::ExactTime<JointState, Contact>;
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

    protected:
        void initialize_() override
        {
            RCLCPP_INFO(node_->get_logger(), "Blind State Converter initialized.");
            // TODO: Add subscriptions, publishers, timers here
            // declare parameters with sensible defaults and read their values
            auto contact_topic = node_->declare_parameter<std::string>("contactdetection_topic", "/contact_detection");
            auto joint_states_topic = node_->declare_parameter<std::string>("joint_states_topic", "/joint_states");

            // log the declared parameter values
            RCLCPP_INFO(node_->get_logger(), "contactdetection_topic = %s", contact_topic.c_str());
            RCLCPP_INFO(node_->get_logger(), "joint_states_topic = %s", joint_states_topic.c_str());


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
        std::shared_ptr<message_filters::Subscriber<JointState>> joint_state_sub_;

  };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(message_converter::BlindStateConverter, message_converter::PluginBase)