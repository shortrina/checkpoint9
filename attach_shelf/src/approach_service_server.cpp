/*
approach_service_server.cpp
* Running ROS2 Service
    - service name = /approach_shelf
    To do List
    1. Detect the legs using the laser "intensity" value.
    - If laser detects one leg or none
        return false;
    - If laser detects both legs
        1. /approach_shelf service should publish center position of between
the legs which is transformed by "cart_frame"
        2. Robot move towards the shelf using this TF coordinates.
        3. Robot move forward 30cm more.(get in the shelf)
        6. Lift the shelf.
*/

#include <chrono>
#include <cmath>
#include <cstddef>
#include <geometry_msgs/msg/twist.hpp>
#include <math.h>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
// to pusblish the transform
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
// to listen to the transform time stamp
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// custom service message (GoToLoading.srv)
#include "attach_shelf/srv/go_to_loading.hpp"

using GoToLanding = attach_shelf::srv::GoToLoading;
using namespace std::chrono_literals;

class ApproachServiceServer : public rclcpp::Node {
public:
  ApproachServiceServer() : Node("approach_service_server_node") {

    navi_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    navigation_options.callback_group = navi_callback_group_;

    // Create subscribers and publishers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ApproachServiceServer::scan_callback, this,
                  std::placeholders::_1),
        navigation_options);

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);

    // Create service
    server_ = this->create_service<attach_shelf::srv::GoToLoading>(
        "approach_shelf",
        std::bind(&ApproachServiceServer::service_callback, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, navi_callback_group_);

    // Create tf_broadcast timer callback for publishing velocity commands
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
#if 0
    tf_broadcast_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ApproachServiceServer::broadcast_timer_callback, this),
        navi_callback_group_);
#endif
    odom_frame_ = "odom";
    laser_frame_ = "robot_front_laser_base_link";
    cart_frame_ = "cart_frame";

    left_leg_index_ = 0;
    right_leg_index_ = 0;

    cart_x_ = 0.0;
    cart_y_ = 0.0;
    cart_yaw_ = 0.0;

    move_extra_distance_ = false;
    found_center_position_ = false;
    robot_rotating_done_ = false;
    find_two_legs_ = false;
    start_service_ = false;
  }

private:
  // CallbackGroup variable
  rclcpp::CallbackGroup::SharedPtr navi_callback_group_;

  // ROS2 publish/subscriber callback variable
  rclcpp::SubscriptionOptions navigation_options;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ROS2 Service variable
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr server_;

  // tf2 listener variables
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS2 tf2 broadcaster variable
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_broadcast_timer_;

  // tf2 tstimestamped variable
  geometry_msgs::msg::TransformStamped transformStamped_;

  // frame names
  std::string odom_frame_, laser_frame_, cart_frame_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

  int left_leg_index_, right_leg_index_;

  float cart_x_;
  float cart_y_;
  float cart_yaw_;
  tf2::Quaternion cart_quat_;

  bool start_service_;
  bool move_extra_distance_;
  bool found_center_position_;
  bool robot_rotating_done_;
  bool find_two_legs_;

  // robot and cart pose
  geometry_msgs::msg::Pose cart_pose_, robot_pose_;

  bool finding_shelf_legs() {
    std::vector<size_t> first_laser_intensity_index;
    for (size_t i = 0; i < last_scan_->intensities.size(); i++) {
      if (last_scan_->intensities[i] >= 7000.0) {
        first_laser_intensity_index.push_back(i);

        /*RCLCPP_INFO(this->get_logger(),
                    "first_intensity_index : %ld, value : %d", i,
                    first_laser_intensity_index[i]);*/
      }
    } // end for

    /*RCLCPP_INFO(this->get_logger(), "first_intensity_index : %d",
                first_laser_intensity_index.size());*/
    // laser index is distances from each other. rages.size ==
    // intensities.size()==1081
    std::vector<std::vector<size_t>> object_counter;
    std::vector<size_t> second_laser_intensity_index;

    second_laser_intensity_index.push_back(first_laser_intensity_index[0]);
    for (size_t i = 1; i < first_laser_intensity_index.size(); i++) {

      if (first_laser_intensity_index[i] - first_laser_intensity_index[i - 1] <
          5) {
        // this object is same.
        second_laser_intensity_index.push_back(first_laser_intensity_index[i]);
      } else {
        object_counter.push_back(second_laser_intensity_index);
        second_laser_intensity_index.clear();
        second_laser_intensity_index.push_back(first_laser_intensity_index[i]);
      }
    }
    if (!second_laser_intensity_index.empty()) {
      object_counter.push_back(second_laser_intensity_index);
    }

    /*RCLCPP_INFO(this->get_logger(),
                "second_intensity_index_size : %d, object_counter_size : %d",
                second_laser_intensity_index.size(), object_counter.size());*/

    if ((object_counter.empty()) || (object_counter.size() == 1)) {
      RCLCPP_ERROR(this->get_logger(),
                   "[finding_shelf_legs] Failed to find legs ");
      return false;
    } else if (object_counter.size() == 2) {
      left_leg_index_ = object_counter[0][0];
      right_leg_index_ = object_counter[1][0];
      find_two_legs_ = true;
      return true;
    }
  }

  void finding_center_position() {
    double left_shelf_distance_from_robot = last_scan_->ranges[left_leg_index_];
    double right_shelf_distance_from_robot =
        last_scan_->ranges[right_leg_index_];
    float cart_magnitude = 0.0;

#if 0
    RCLCPP_INFO(this->get_logger(), "left_distance : %lf",
                left_shelf_distance_from_robot);
    RCLCPP_INFO(this->get_logger(), "right_distance : %lf",
                right_shelf_distance_from_robot);
    RCLCPP_INFO(this->get_logger(), "gab_distance : %lf",
                left_shelf_distance_from_robot -
                    right_shelf_distance_from_robot);
#endif

    // The rays are from 0 to 1080 clockwise [225 to -45 degrees]
    cart_magnitude =
        (left_shelf_distance_from_robot + right_shelf_distance_from_robot) /
        2.0;
    // Calculate the average angle between the two legs
    cart_yaw_ = last_scan_->angle_min + (left_leg_index_ + right_leg_index_) /
                                            2.0 * last_scan_->angle_increment;
    // Calculate the cartesian coordinates
    cart_x_ = cart_magnitude * cos(cart_yaw_);
    cart_y_ = cart_magnitude * sin(cart_yaw_);
    cart_quat_.setRPY(0.0, 0.0, cart_yaw_);

    // adding_fixed_cartframe();
    found_center_position_ = true;
  }

  void adding_fixed_cartframe() {
#if 1
    //------------for take the value of time stamp--------------
    geometry_msgs::msg::TransformStamped receiver_stamped;
    try {
      receiver_stamped = tf_buffer_->lookupTransform(odom_frame_, laser_frame_,
                                                     tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                  odom_frame_.c_str(), laser_frame_.c_str(), ex.what());
    }
#endif
    //------------for take the value of time stamp--------------
    geometry_msgs::msg::TransformStamped cart_transform;

    // cart_transform.header.stamp = receiver_stamped.header.stamp;
    cart_transform.header.stamp = this->get_clock()->now();
    cart_transform.header.frame_id = laser_frame_;
    cart_transform.child_frame_id = cart_frame_;

    // Set initial position
    cart_transform.transform.translation.x = cart_x_;
    cart_transform.transform.translation.y = cart_y_;
    cart_transform.transform.translation.z = 0.0;

    // Set initial rotation (identity quaternion)
    cart_transform.transform.rotation.x = cart_quat_.x();
    cart_transform.transform.rotation.y = cart_quat_.y();
    cart_transform.transform.rotation.z = cart_quat_.z();
    cart_transform.transform.rotation.w = cart_quat_.w();

    // Broadcast the static transform
    tf_broadcaster_->sendTransform(cart_transform);

    RCLCPP_INFO(this->get_logger(),
                "[adding_fixed_cartframe] Broadcasting attach cart frame");
  }

  void move_to_cart_center() {
    geometry_msgs::msg::Twist cmd_vel_msg;
    double distance_to_cart = sqrt(pow(cart_x_, 2) + pow(cart_y_, 2));
    if (distance_to_cart > 0.4) {
      RCLCPP_INFO(this->get_logger(), "Moving to cart_frame");
      cmd_vel_msg.linear.x = 0.1;
      cmd_vel_msg.angular.z = 0.1;
      cmd_vel_pub_->publish(cmd_vel_msg);
      found_center_position_ = false;
    } else {
      RCLCPP_INFO(this->get_logger(), "Stop to moving cart");
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd_vel_msg);
      move_extra_distance_ = true;
    }
  }

  void rotating_center_cart() {
    geometry_msgs::msg::Twist cmd_vel_msg;
    if (robot_pose_.orientation.z >= 0.05) {
      RCLCPP_INFO(this->get_logger(), "Rotating left to cart_frame");
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = -0.1;
      cmd_vel_pub_->publish(cmd_vel_msg);
    } else if (robot_pose_.orientation.z <= -0.05) {
      RCLCPP_INFO(this->get_logger(), "Rotating right to moving cart");
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = 0.1;
      cmd_vel_pub_->publish(cmd_vel_msg);
    } else if (((robot_pose_.orientation.z > 0.0) &&
                (robot_pose_.orientation.z <= 0.05)) ||
               ((robot_pose_.orientation.z >= -0.05) &&
                (robot_pose_.orientation.z <= 0.0))) {
      RCLCPP_INFO(this->get_logger(), "Stoping to moving cart");
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd_vel_msg);
      robot_rotating_done_ = true;
    }
  }

  void moving_under_cart() {
    geometry_msgs::msg::Twist cmd_vel_msg;
    int count = 3;

    while (count != 0) {
      cmd_vel_msg.linear.x = 0.1;
      cmd_vel_msg.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd_vel_msg);
      count--;
    }
  }

  /*find cart_legs and publish tf frame, and moving under the cart */
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_scan_ = msg;

    if (finding_shelf_legs()) {
      adding_fixed_cartframe();
      // RCLCPP_INFO(this->get_logger(), "Success finding Shelf two legs ");
      if (!found_center_position_) {
        finding_center_position();
      }
      if (found_center_position_ && !move_extra_distance_) {
        move_to_cart_center();
      }

      if (move_extra_distance_ && !robot_rotating_done_) {
        rotating_center_cart();
      }

      if (robot_rotating_done_) {
        moving_under_cart();
      }
    }
  } // end of scan_callback

  /*Get request from client*/
  void service_callback(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Response>
          response) {
    // TODO: implement your service here
    if (request->attach_to_shelf) {
      if (find_two_legs_) {
        response->complete = true;
      } else {
        RCLCPP_INFO(this->get_logger(), "failed to find 2 legs");
        response->complete = false;
      }
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "failed to get request value from client");
      response->complete = false;
    }

  } // end_of_service_callback

#if 0
  void broadcast_timer_callback() {
    rclcpp::Time now = this->get_clock()->now();
    double x = now.seconds() * PI;

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = now;
    t.header.frame_id = "odom";
    t.child_frame_id = "cart_frame";
    t.transform.translation.x = 5.5;       // 5.315288;
    t.transform.translation.y = -0.8;      //-2.888712;
    t.transform.translation.z = 0.0025002; // 0.0025002;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 1.578367; // 1.578367;
    t.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(t);
  } // end of broadcast_timer_callback
#endif
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<ApproachServiceServer> approach_service =
      std::make_shared<ApproachServiceServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(approach_service);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
