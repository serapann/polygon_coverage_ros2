#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

// int main(int argc, char * argv[]) {
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("marker_publisher");

//   auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
//     "path1_markers", 10); // 创建发布者，发布到 "path_markers" 话题

//   // 创建一个 Marker 消息
//   auto marker_msg = std::make_shared<visualization_msgs::msg::Marker>();
//   marker_msg->header.frame_id = "world";
//   marker_msg->header.stamp = node->now();
//   marker_msg->ns = "polygonhull";
//   marker_msg->id = 0;
//   marker_msg->type = visualization_msgs::msg::Marker::LINE_STRIP;
//   marker_msg->action = visualization_msgs::msg::Marker::ADD;
//   marker_msg->pose.position.x = 0.0;
//   marker_msg->pose.position.y = 0.0;
//   marker_msg->pose.position.z = 0.0;
//   marker_msg->pose.orientation.x = 0.0;
//   marker_msg->pose.orientation.y = 0.0;
//   marker_msg->pose.orientation.z = 0.0;
//   marker_msg->pose.orientation.w = 1.0;
//   marker_msg->scale.x = 0.1;
//   marker_msg->scale.y = 0.1;
//   marker_msg->scale.z = 0.0;
//   marker_msg->color.r = 0.0;
//   marker_msg->color.g = 1.0;
//   marker_msg->color.b = 0.0;
//   marker_msg->color.a = 1.0;
//   marker_msg->lifetime = rclcpp::Duration(0, 0);
//   marker_msg->frame_locked = false;

//   // 添加多边形的顶点
//  // 添加多边形的顶点
//   geometry_msgs::msg::Point point;
//   point.x = 0.0;
//   point.y = 0.0;
//   point.z = 0.5;
//   marker_msg->points.push_back(point);

//   point.x = 100.0;
//   point.y = 0.0;
//   point.z = 0.5;
//   marker_msg->points.push_back(point);

//   point.x = 100.0;
//   point.y = 100.0;
//   point.z = 0.5;
//   marker_msg->points.push_back(point);

//   point.x = 0.0;
//   point.y = 100.0;
//   point.z = 0.5;
//   marker_msg->points.push_back(point);

//   point.x = 0.0;
//   point.y = 0.0;
//   point.z = 0.5;
//   marker_msg->points.push_back(point);


// visualization_msgs::msg::Marker polygon_msg;
// polygon_msg.header.frame_id = "world";
// polygon_msg.header.stamp = node->now();
// polygon_msg.ns = "polygonhole_0";
// polygon_msg.id = 1;
// polygon_msg.type = visualization_msgs::msg::Marker::LINE_STRIP; // 使用TRIANGLE_LIST绘制多边形
// polygon_msg.action = visualization_msgs::msg::Marker::ADD;
// polygon_msg.pose.position.x = 0.0;
// polygon_msg.pose.position.y = 0.0;
// polygon_msg.pose.position.z = 0.0;
// polygon_msg.pose.orientation.x = 0.0;
// polygon_msg.pose.orientation.y = 0.0;
// polygon_msg.pose.orientation.z = 0.0;
// polygon_msg.pose.orientation.w = 1.0;
// polygon_msg.scale.x = 1.0;
// polygon_msg.scale.y = 1.0;
// polygon_msg.scale.z = 1.0;
// polygon_msg.color.r = 1.0; // 填充颜色的红色分量
// polygon_msg.color.g = 0.0; // 填充颜色的绿色分量
// polygon_msg.color.b = 0.0; // 填充颜色的蓝色分量
// polygon_msg.color.a = 1.0; // 透明度
// polygon_msg.lifetime = rclcpp::Duration(0, 0);
// polygon_msg.frame_locked = false;


// point.x = 55.66013134894843;
// point.y = 55.65120076448767;
// point.z = 4.0;
// polygon_msg.points.push_back(point);

// point.x = 45.0961855134036;
// point.y = 25.92900939815026;
// point.z = 4.0;
// polygon_msg.points.push_back(point);

// point.x = 73.0;
// point.y = 34.24070455081452;
// point.z = 4.0;
// polygon_msg.points.push_back(point);

// // 通过重复第一个点来闭合第一个多边形
// polygon_msg.points.push_back(polygon_msg.points[0]);

// visualization_msgs::msg::Marker polygon_msg1;
// polygon_msg1.header.frame_id = "world";
// polygon_msg1.header.stamp = node->now();
// polygon_msg1.ns = "polygonhole_1";
// polygon_msg1.id = 1;
// polygon_msg1.type = visualization_msgs::msg::Marker::LINE_STRIP; // 使用TRIANGLE_LIST绘制多边形
// polygon_msg1.action = visualization_msgs::msg::Marker::ADD;
// polygon_msg1.pose.position.x = 0.0;
// polygon_msg1.pose.position.y = 0.0;
// polygon_msg1.pose.position.z = 0.0;
// polygon_msg1.pose.orientation.x = 0.0;
// polygon_msg1.pose.orientation.y = 0.0;
// polygon_msg1.pose.orientation.z = 0.0;
// polygon_msg1.pose.orientation.w = 1.0;
// polygon_msg1.scale.x = 1.0;
// polygon_msg1.scale.y = 1.0;
// polygon_msg1.scale.z = 1.0;
// polygon_msg1.color.r = 1.0; // 填充颜色的红色分量
// polygon_msg1.color.g = 0.0; // 填充颜色的绿色分量
// polygon_msg1.color.b = 0.0; // 填充颜色的蓝色分量
// polygon_msg1.color.a = 1.0; // 透明度
// polygon_msg1.lifetime = rclcpp::Duration(0, 0);
// polygon_msg1.frame_locked = false;

// // 添加第二个多边形的顶点
// point.x = 10.532926709487015;
// point.y = 65.1078154035655;
// point.z = 4.0;
// polygon_msg1.points.push_back(point);

// point.x = 35.567417803658135;
// point.y = 50.95927935884282;
// point.z = 4.0;
// polygon_msg1.points.push_back(point);

// point.x = 22.00327751845971;
// point.y = 84.36805971116891;
// point.z = 4.0;
// polygon_msg1.points.push_back(point);

// // 通过重复第一个点来闭合第二个多边形
// polygon_msg1.points.push_back(polygon_msg1.points[0]);

//   // 发布新的Marker消息
//   while (rclcpp::ok()) {
//       marker_pub->publish(*marker_msg);
//       marker_pub->publish(polygon_msg);
//       marker_pub->publish(polygon_msg1);
//       rclcpp::spin_some(node);
//   }


//   rclcpp::shutdown();
//   return 0;
// }


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("marker_publisher");

  // // 配置QoS
  // rclcpp::QoS qos(rclcpp::KeepLast(10)); // 保留最近的10条消息
  // qos.transient_local(); // 使用本地传输
  // // qos.reliability(rclcpp::QoS::ReliabilityPolicy::RELIABLE); // 使用可靠的消息传输
  // qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE); // 使用可靠的消息传输


  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
    "visualization_marker", 10); // 创建发布者，发布到 "visualization_marker" 话题

  // 创建第一个 Marker 消息
  visualization_msgs::msg::Marker marker_msg1;
  marker_msg1.header.frame_id = "world";
  marker_msg1.header.stamp = node->now();
  marker_msg1.ns = "polygonhull";
  marker_msg1.id = 0;
  marker_msg1.type = visualization_msgs::msg::Marker::LINE_STRIP; // 设置为 CUBE 类型
  marker_msg1.action = visualization_msgs::msg::Marker::ADD;
  marker_msg1.pose.position.x = 0.0;
  marker_msg1.pose.position.y = 0.0;
  marker_msg1.pose.position.z = 0.0;
  marker_msg1.pose.orientation.x = 0.0;
  marker_msg1.pose.orientation.y = 0.0;
  marker_msg1.pose.orientation.z = 0.0;
  marker_msg1.pose.orientation.w = 1.0;
  marker_msg1.scale.x = 0.1;
  marker_msg1.scale.y = 0.1;
  marker_msg1.scale.z = 0.1;
  marker_msg1.color.r = 0.0;
  marker_msg1.color.g = 1.0;
  marker_msg1.color.b = 0.0;
  marker_msg1.color.a = 1.0;
  marker_msg1.lifetime = rclcpp::Duration(0, 0);
  marker_msg1.frame_locked = false;

  // 添加第一个 Marker 消息的顶点
  geometry_msgs::msg::Point point;
  point.x = 0.0;
  point.y = 0.0;
  point.z = 0.5;
  marker_msg1.points.push_back(point);

  point.x = 100.0;
  point.y = 0.0;
  point.z = 0.5;
  marker_msg1.points.push_back(point);

  point.x = 100.0;
  point.y = 100.0;
  point.z = 0.5;
  marker_msg1.points.push_back(point);

  point.x = 0.0;
  point.y = 100.0;
  point.z = 0.5;
  marker_msg1.points.push_back(point);

  point.x = 0.0;
  point.y = 0.0;
  point.z = 0.5;
  marker_msg1.points.push_back(point);

  // 发布第一个 Marker 消息
  marker_pub->publish(marker_msg1);

  // 创建第三个 Marker 消息
  visualization_msgs::msg::Marker marker_msg3;
  marker_msg3.header.frame_id = "world";
  marker_msg3.header.stamp = node->now();
  marker_msg3.ns = "polygonhole_mesh_0";
  marker_msg3.id = 1;
  marker_msg3.type = visualization_msgs::msg::Marker::TRIANGLE_LIST; // 设置为 MESH_RESOURCE 类型
  marker_msg3.action = visualization_msgs::msg::Marker::ADD;
  marker_msg3.pose.position.x = 0.0;
  marker_msg3.pose.position.y = 0.0;
  marker_msg3.pose.position.z = 0.0;
  marker_msg3.pose.orientation.x = 0.0;
  marker_msg3.pose.orientation.y = 0.0;
  marker_msg3.pose.orientation.z = 0.0;
  marker_msg3.pose.orientation.w = 1.0;
  marker_msg3.scale.x = 1.0;
  marker_msg3.scale.y = 1.0;
  marker_msg3.scale.z = 1.0;
  marker_msg3.color.r = 1.0;
  marker_msg3.color.g = 0.0;
  marker_msg3.color.b = 0.0;
  marker_msg3.color.a = 1.0;
  marker_msg3.lifetime = rclcpp::Duration(0, 0);
  marker_msg3.frame_locked = false;

  // 添加第三个 Marker 消息的顶点
  point.x = 20.0;
  point.y = 38.76409597325997;
  point.z = 0.5;
  marker_msg3.points.push_back(point);

  point.x = 45.0961855134036;
  point.y = 25.92900939815026;
  point.z = 0.5;
  marker_msg3.points.push_back(point);

  point.x = 73.0;
  point.y = 34.24070455081452;
  point.z = 0.5;
  marker_msg3.points.push_back(point);

  // 发布第三个 Marker 消息
  marker_pub->publish(marker_msg3);

  // 创建第四个 Marker 消息
  visualization_msgs::msg::Marker marker_msg4;
  marker_msg4.header.frame_id = "world";
  marker_msg4.header.stamp = node->now();
  marker_msg4.ns = "polygonhole_mesh_1";
  marker_msg4.id = 2;
  marker_msg4.type = visualization_msgs::msg::Marker::TRIANGLE_LIST; // 设置为 MESH_RESOURCE 类型
  marker_msg4.action = visualization_msgs::msg::Marker::ADD;
  marker_msg4.pose.position.x = 0.0;
  marker_msg4.pose.position.y = 0.0;
  marker_msg4.pose.position.z = 0.0;
  marker_msg4.pose.orientation.x = 0.0;
  marker_msg4.pose.orientation.y = 0.0;
  marker_msg4.pose.orientation.z = 0.0;
  marker_msg4.pose.orientation.w = 1.0;
  marker_msg4.scale.x = 1.0;
  marker_msg4.scale.y = 1.0;
  marker_msg4.scale.z = 1.0;
  marker_msg4.color.r = 1.0;
  marker_msg4.color.g = 0.0;
  marker_msg4.color.b = 0.0;
  marker_msg4.color.a = 1.0;
  marker_msg4.lifetime = rclcpp::Duration(0, 0);
  marker_msg4.frame_locked = false;

  // 添加第四个 Marker 消息的顶点
  point.x = 10.532926709487015;
  point.y = 65.1078154035655;
  point.z = 0.5;
  marker_msg4.points.push_back(point);

  point.x = 35.56741780365814;
  point.y = 50.95927935884282;
  point.z = 0.5;
  marker_msg4.points.push_back(point);

  point.x = 22.00327751845971;
  point.y = 84.36805971116891;
  point.z = 0.5;
  marker_msg4.points.push_back(point);

  // 发布第四个 Marker 消息
  // if(rclcpp::ok()){
  //   marker_pub->publish(marker_msg1);
  //   marker_pub->publish(marker_msg1); 
  //   rclcpp::spin_some(node);
  // }
  int cost_function_type_4;
  node->declare_parameter("cost_function_type", 4);
  node->get_parameter("cost_function_type", cost_function_type_4);
  RCLCPP_INFO(node->get_logger(), "cost_function_type_4: %d", cost_function_type_4);
  while (rclcpp::ok()) {
  // 发布marker1
  for(int i = 0;i<10;i++)
  {
       marker_pub->publish(marker_msg1);
       marker_pub->publish(marker_msg3);
       marker_pub->publish(marker_msg4);
       RCLCPP_INFO(node->get_logger(), "suceess_第%d次！",i);
      //  break;
  }
  break;
  // marker_pub->publish(marker_msg1);

  // marker_pub->publish(marker_msg3);

  // marker_pub->publish(marker_msg4);

  rclcpp::spin_some(node);
  }  

  rclcpp::shutdown();
  return 0;
}
