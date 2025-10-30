#include <message_converter/converter_base.hpp>
#include <cmath>
#include <sensor_msgs/msg/joint_state.hpp>
#include <state_estimator_msgs/msg/contact_detection.hpp>
#include <dls2_msgs/msg/base_state_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <message_filters/time_synchronizer.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>



namespace message_converter
{
using Odom = nav_msgs::msg::Odometry;

  using IMU = sensor_msgs::msg::Imu;
  using BaseState = dls2_msgs::msg::BaseStateMsg;
  using ApproximateTimePolicy = message_filters::sync_policies::ApproximateTime<Odom, IMU>;
using ExactTimePolicy = message_filters::sync_policies::ExactTime<Odom, IMU>;
  #define MySyncPolicy ApproximateTimePolicy
  class BaseStateConverter : public message_converter::PluginBase
  {

    public:

        BaseStateConverter() = default;

        ~BaseStateConverter() override 
        {
          RCLCPP_INFO(node_->get_logger(), "Base State Converter shutting down.");
        }

        std::string getName() override
        {
          return std::string("Base State Converter");
        }
        std::string getDescription() override
        {
          return std::string("A converter that aggregates odometry, joint states and legodometry to produce state estimation");
        }

        void syncCallback(const Odom::ConstSharedPtr &odom, const IMU::ConstSharedPtr &imu)
        {

            //Set float timestamp
            msg_.timestamp = odom->header.stamp.sec + odom->header.stamp.nanosec * 1e-9;
            //set frame ID
            msg_.frame_id = odom->header.frame_id;

            msg_.position[0] = odom->pose.pose.position.x;
            msg_.position[1] = odom->pose.pose.position.y;
            msg_.position[2] = odom->pose.pose.position.z;
            msg_.orientation[0] = odom->pose.pose.orientation.x;
            msg_.orientation[1] = odom->pose.pose.orientation.y;
            msg_.orientation[2] = odom->pose.pose.orientation.z;
            msg_.orientation[3] = odom->pose.pose.orientation.w;

            msg_.linear_velocity[0] = odom->twist.twist.linear.x;
            msg_.linear_velocity[1] = odom->twist.twist.linear.y;
            msg_.linear_velocity[2] = odom->twist.twist.linear.z;
            msg_.angular_velocity[0] = imu->angular_velocity.x;
            msg_.angular_velocity[1] = imu->angular_velocity.y;
            msg_.angular_velocity[2] = imu->angular_velocity.z;

            msg_.linear_acceleration[0] = imu->linear_acceleration.x;
            msg_.linear_acceleration[1] = imu->linear_acceleration.y;
            msg_.linear_acceleration[2] = imu->linear_acceleration.z;

            //fill angular acceleration with zeros for now
            msg_.angular_acceleration[0] = 0.0;
            msg_.angular_acceleration[1] = 0.0;
            msg_.angular_acceleration[2] = 0.0;

            base_state_pub_->publish(msg_);
        }

    protected:
        void initialize_() override
        {
            auto node = this->node_;
            RCLCPP_INFO(node_->get_logger(), "Base State Converter initialized.");
            // TODO: Add subscriptions, publishers, timers here
            // declare parameters with sensible defaults and read their values
            auto odom_topic = node_->declare_parameter<std::string>("odom_topic", "/odom");
            auto imu_topic = node_->declare_parameter<std::string>("imu_topic", "/imu");
            auto pub_topic = node_->declare_parameter<std::string>("pub_topic", "/base_state_topic");
            auto sync_tolerance_ms = node_->declare_parameter<int>("sync_tolerance_ms", 50);

            //Subscribers
            auto qos = rclcpp::SensorDataQoS();

            joint_state_sub_ = std::make_shared<message_filters::Subscriber<Odom>>(node, odom_topic, qos);
            contact_sub_ = std::make_shared<message_filters::Subscriber<IMU>>(node, imu_topic, qos);

            //Synchronizer
            sync_ =  std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
                MySyncPolicy(10), *joint_state_sub_, *contact_sub_);
            sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_tolerance_ms / 1000.0));
            sync_->registerCallback(
                std::bind(&BaseStateConverter::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

            //Publisher
            base_state_pub_ = node_->create_publisher<BaseState>(pub_topic, qos);
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
        std::shared_ptr<message_filters::Subscriber<Odom>> joint_state_sub_;
        std::shared_ptr<message_filters::Subscriber<IMU>> contact_sub_;
        std::shared_ptr<message_filters::Synchronizer<ApproximateTimePolicy>> sync_;

        rclcpp::Publisher<BaseState>::SharedPtr base_state_pub_;
        BaseState msg_;

  };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(message_converter::BaseStateConverter, message_converter::PluginBase)