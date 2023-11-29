#include"test1/test1.h"
namespace test{
    test1::test1(const std::shared_ptr<rclcpp::Node> &nh)
    {
        RCLCPP_INFO(rclcpp::get_logger("coverage_planner111"), "coverage_planner节点已经启动1111.");
        
    }
}