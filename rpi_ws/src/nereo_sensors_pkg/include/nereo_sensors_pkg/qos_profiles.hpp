#ifndef QOS_PROFILES_HPP
#define QOS_PROFILES_HPP

#include "rclcpp/rclcpp.hpp"

static rclcpp::QoS getSensorQoS() {
  rclcpp::QoS sensor_qos(1);
  sensor_qos.best_effort();
  sensor_qos.volatile_durability();
  sensor_qos.keep_last();
  return sensor_qos;
}

static rclcpp::QoS getReliableQoS() {
  rclcpp::QoS reliable_qos(5);
  reliable_qos.reliable();
  reliable_qos.volatile_durability();
  reliable_qos.keep_last();
  return reliable_qos;
}

#endif  // QOS_PROFILES_HPP
