#ifndef CMDVELCOMMAND_H
#define CMDVELCOMMAND_H

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/time.hpp>

struct CmdVelCommand {
    geometry_msgs::msg::Twist twist;
    rclcpp::Time stamp;
    bool valid{false};
};

#endif // CMDVELCOMMAND_H