#ifndef ODOMETRY_SENSOR_MODEL_HPP_
#define ODOMETRY_SENSOR_MODEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "sensor_model.hpp"

namespace localization {

class OdometrySensorModel : public SensorModel {
    public:
        explicit OdometrySensorModel(rclcpp::Node &node);
    
        void UpdateMeasurement(const nav_msgs::msg::Odometry::SharedPtr msg);
        bool IsMeasurementAvailable(const rclcpp::Time &curr_time) override;
        double ComputeLogProb(const Particle &particle) override;
        double ComputeLogNormalizer() override;

    private:
        nav_msgs::msg::Odometry last_msg_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}

#endif