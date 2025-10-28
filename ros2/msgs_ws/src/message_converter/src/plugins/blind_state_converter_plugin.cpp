#include <message_converter/converter_base.hpp>
#include <cmath>

namespace message_converter
{
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


        void initialize_() override
        {
            node_ = std::move(node_);
            RCLCPP_INFO(node_->get_logger(), "Blind State Converter initialized.");
        }

        void shutdown_() override { }
		void pause_() override { }
		void resume_() override { }
		void reset_() override { }

    private:
    	bool paused_;
    	bool running_;
    	bool initialized_;
  };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(message_converter::BlindStateConverter, message_converter::PluginBase)