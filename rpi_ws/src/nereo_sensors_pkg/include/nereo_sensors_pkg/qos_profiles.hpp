#ifndef QOS_PROFILES_HPP
#define QOS_PROFILES_HPP

#include "rclcpp/rclcpp.hpp"

static rclcpp::QoS getSensorQoS() {
  rclcpp::QoS sensor_qos(1);
  sensor_qos.best_effort();
  sensor_qos.durability_volatile();
  sensor_qos.keep_last(10);
  return sensor_qos;
}

static rclcpp::QoS getReliableQoS() {
  rclcpp::QoS reliable_qos(5);
  reliable_qos.reliable();
  reliable_qos.durability_volatile();
  reliable_qos.keep_last(10);
  return reliable_qos;
}

#endif  // QOS_PROFILES_HPP
