#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "co_3dto2d_mapping/occupancy_mapper_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ToyOccupancyMapper>());
  rclcpp::shutdown();
  return 0;
}
