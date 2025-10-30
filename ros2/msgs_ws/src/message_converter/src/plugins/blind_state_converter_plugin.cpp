#include <message_converter/converter_base.hpp>
#include <cmath>
#include <sensor_msgs/msg/joint_state.hpp>
#include <state_estimator_msgs/msg/contact_detection.hpp>
#include <dls2_msgs/msg/blind_state_msg.hpp>

#include <message_filters/time_synchronizer.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>



namespace message_converter
{
	using JointState = sensor_msgs::msg::JointState;
  using Contact = state_estimator_msgs::msg::ContactDetection;
  using BlindData = dls2_msgs::msg::BlindStateMsg;
  using ApproximateTimePolicy = message_filters::sync_policies::ApproximateTime<JointState, Contact>;
	using ExactTimePolicy = message_filters::sync_policies::ExactTime<JointState, Contact>;
  #define MySyncPolicy ApproximateTimePolicy
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

        void syncCallback(const JointState::ConstSharedPtr &joint_state, const Contact::ConstSharedPtr &contact)
        {
            RCLCPP_INFO(node_->get_logger(), "Synchronized JointState and ContactDetection received.");

            // publishing
			      msg_.timestamp = node_->get_clock()->now();
			      msg_.frame_id = "base_link";
            //joint state values
            msg_.joints_name = joint_state->name;
            msg_.joints_position = joint_state->position;
            msg_.joints_velocity = joint_state->velocity;
            msg_.joints_effort = joint_state->effort;
            //TODO: Calculate acceleration? Set to 0 for now
            //TODO: Only use temperature if necessary
            //TODO see if this is even used (doesnt seem like it)
            //msg_.feet_contact = contact->

            blind_state_pub_->publish(msg_);




        }

    protected:
        void initialize_() override
        {
            auto node = this->node_;
            RCLCPP_INFO(node_->get_logger(), "Blind State Converter initialized.");
            // TODO: Add subscriptions, publishers, timers here
            // declare parameters with sensible defaults and read their values
            auto contact_topic = node_->declare_parameter<std::string>("contactdetection_topic", "/contact_detection");
            auto joint_states_topic = node_->declare_parameter<std::string>("joint_states_topic", "/joint_states");
            auto pub_topic = node_->declare_parameter<std::string>("pub_topic", "/blind_state_topic");
            auto sync_tolerance_ms = node_->declare_parameter<int>("sync_tolerance_ms", 200);

            //Subscribers
            auto qos = rclcpp::SensorDataQoS();

            joint_state_sub_ = std::make_shared<message_filters::Subscriber<JointState>>(node, joint_states_topic,qos);
            contact_sub_ = std::make_shared<message_filters::Subscriber<Contact>>(node, contact_topic,qos);

            //Synchronizer
            sync_ =  std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
                MySyncPolicy(10), *joint_state_sub_, *contact_sub_);
            sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_tolerance_ms / 1000.0));
            sync_->registerCallback(
                std::bind(&BlindStateConverter::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

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
        std::shared_ptr<message_filters::Subscriber<JointState>> joint_state_sub_;
        std::shared_ptr<message_filters::Subscriber<Contact>> contact_sub_;
        std::shared_ptr<message_filters::Synchronizer<ApproximateTimePolicy>> sync_;

        rclcpp::Publisher<BlindData>::SharedPtr blind_state_pub_;
        BlindData msg_;

  };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(message_converter::BlindStateConverter, message_converter::PluginBase)