#ifndef message_converter_NODE_HPP
#define message_converter_NODE_HPP


#include <boost/make_unique.hpp>
#include<boost/assign/list_of.hpp>
#include<boost/assert.hpp>
#include<boost/date_time/posix_time/posix_time.hpp>
#include<boost/thread/thread.hpp>
#include<boost/thread/mutex.hpp>
#include<boost/thread/shared_mutex.hpp>
#include <boost/chrono.hpp>
#include <atomic>

#include <inttypes.h>
#include <stdint.h>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <iomanip>
#include <pluginlib/class_loader.hpp>
#include <fnmatch.h>
#include <memory>
#include <stdexcept>


#include "converter_base.hpp"






namespace message_converter {

class message_converter_node {
public:
    explicit message_converter_node(rclcpp::Node::SharedPtr node);
    void shutdown();
    void time_out_thread();
    void stop_all_threads();

protected:
    rclcpp::Node::SharedPtr node_;
    pluginlib::ClassLoader<message_converter::PluginBase> plugin_loader;
    std::vector<std::shared_ptr<message_converter::PluginBase>> loaded_plugins;
    // ROS2 services to be added during migration


    void readParamsFromROSParameterServer();
    bool add_plugin(std::string &name);
    bool add_plugin(std::string &pl_name, std::vector<std::string> &blacklist, std::vector<std::string> &whitelist);
    bool is_blacklisted(std::string &pl_name, std::vector<std::string> &blacklist, std::vector<std::string> &whitelist);
    bool pattern_match(std::string &pattern, std::string &pl_name);
    inline bool getWhitelist(std::vector<std::string> &whitelist);
    inline bool getBlacklist(std::vector<std::string> &blacklist);
};



} //namespace message_converter

#endif
