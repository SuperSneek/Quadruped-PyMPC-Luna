

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "message_converter/message_converter.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

namespace message_converter
{
  void message_converter_node::shutdown() {
    RCLCPP_INFO(node_->get_logger(), "Message Converter node: Shutting node down...");
    for (auto const &plugin : loaded_plugins) {
        RCLCPP_INFO(node_->get_logger(), "Shutting down %s...", plugin->getName().c_str());
        plugin->shutdown();
        RCLCPP_INFO(node_->get_logger(), "Done");
    }
    RCLCPP_INFO(node_->get_logger(), "Message Converter node shut down.");
  }

  message_converter_node::message_converter_node(rclcpp::Node::SharedPtr node)
      : node_(std::move(node)), plugin_loader("message_converter", "message_converter::PluginBase") {

      for (auto &name : plugin_loader.getDeclaredClasses()) {
          add_plugin(name);
      }

      RCLCPP_INFO(node_->get_logger(), "Message converter node initialized.");
  }

  bool message_converter_node::add_plugin(std::string &pl_name) {
    try {
        auto plugin = plugin_loader.createSharedInstance(pl_name);
        RCLCPP_INFO(node_->get_logger(), "Plugin %s loaded", pl_name.c_str());
        plugin->initialize(node_);
        RCLCPP_INFO(node_->get_logger(), "Plugin %s initialized", pl_name.c_str());
        loaded_plugins.push_back(plugin);
        return true;
    } catch (pluginlib::PluginlibException &ex) {
        RCLCPP_ERROR(node_->get_logger(), "Plugin %s load exception: %s", pl_name.c_str(), ex.what());
    } catch (std::exception &ex) {
        RCLCPP_ERROR(node_->get_logger(), "Plugin %s load exception: %s", pl_name.c_str(), ex.what());
    } catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "Plugin %s load exception.", pl_name.c_str());
    }
    return false;
  }

} // namespace message_converter

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>("message_converter");
	auto se = std::make_shared<message_converter::message_converter_node>(node);
	rclcpp::spin(node);
	se->shutdown();
	rclcpp::shutdown();
	return 0;
}