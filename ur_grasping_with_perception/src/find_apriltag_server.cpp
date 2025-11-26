#include <functional>
#include <memory>
#include <thread>

#include "grasping_with_perception_interfaces/action/findapriltag.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "ur_grasping_with_perception/visibility_control.h"

#include "apriltag_msgs/msg/april_tag_detection_array.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "geometry_msgs/msg/point.hpp"//nouvelle inclusion 
#include <cmath> // pour utiliser std::round

namespace ur_grasping_with_perception
{
class FindApriltagActionServer : public rclcpp::Node
{
public:
  using FindApriltag= grasping_with_perception_interfaces::action::Findapriltag;
  using GoalHandleFindApriltag = rclcpp_action::ServerGoalHandle<FindApriltag>;

  ACTION_FIND_APRILTAG_PUBLIC
  explicit FindApriltagActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("findapriltag_action_server", options)
  {
    using namespace std::placeholders;
    
    pixel_coordinates_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("apriltag_pixel_coordinates", 10);//<!> ligne ajoutée

    this->action_server_ = rclcpp_action::create_server<FindApriltag>(
      this,
      "findapriltag",
      std::bind(&FindApriltagActionServer::handle_goal, this, _1, _2),
      std::bind(&FindApriltagActionServer::handle_cancel, this, _1),
      std::bind(&FindApriltagActionServer::handle_accepted, this, _1));

    apriltag_subscripber_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    "/detections", 10, std::bind(&FindApriltagActionServer::callbackAprilTagDetection, this, _1));

    target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  rclcpp_action::Server<FindApriltag>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pixel_coordinates_publisher_; //<!> ligne ajoutée
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_subscripber_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
  geometry_msgs::msg::TransformStamped t;
  int32_t tag_id_ = -1;
  bool tag_found = false;
  int32_t nb_tag_found = -1;



  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FindApriltag::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with tag id %d", goal->tag_id);
    (void)uuid;
    tag_id_ = goal->tag_id;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFindApriltag> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFindApriltag> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&FindApriltagActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFindApriltag> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FindApriltag::Feedback>();
    feedback->tracker_status = nb_tag_found;
    //auto & sequence = feedback->partial_sequence;
    //sequence.push_back(0);
    //sequence.push_back(1);
    
    auto result = std::make_shared<FindApriltag::Result>();

    //for (int i = 1; (i < goal->tag_id) && rclcpp::ok(); ++i) {
    while (!tag_found && rclcpp::ok()) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->tag_found = -1.0;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      //sequence.push_back(sequence[i] + sequence[i - 1]);
      feedback->tracker_status = nb_tag_found;
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->tag_found = tag_id_;
      result->pose.translation.x = t.transform.translation.x;
      result->pose.translation.y = t.transform.translation.y;
      result->pose.translation.z = t.transform.translation.z;
      result->pose.rotation.x = t.transform.rotation.x;
      result->pose.rotation.y = t.transform.rotation.y;
      result->pose.rotation.z = t.transform.rotation.z;
      result->pose.rotation.w = t.transform.rotation.w;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  virtual void callbackAprilTagDetection(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
    if (!msg->detections.empty())
    {
       RCLCPP_INFO(this->get_logger(), "Nb apriltag found :%lu", msg->detections.size());
       nb_tag_found = msg->detections.size();
       for (int i=0;i < msg->detections.size(); i++) {
        if (msg->detections[0].id == tag_id_) { 
            tag_found = true;
            break;
        }
       }

        if (tag_found) {

        std::string toFrameRel = target_frame_.c_str();
        std::string fromFrameRel = "object"; 
        try {
          t = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }
       
        RCLCPP_INFO(this->get_logger(), "Object Position :%f %f %f ", t.transform.translation.x,
        t.transform.translation.y,t.transform.translation.z);
        RCLCPP_INFO(this->get_logger(), "Object Orientation :%f %f %f %f", t.transform.rotation.x,
        t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w );

        tf2::Quaternion q(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double r{}, p{}, y{};
        m.getRPY(r, p, y);
        RCLCPP_INFO(this->get_logger(), "Object Orientation :%f %f %f", r,p,y );
       }
       // <!> début du code ajouté
       
       for (size_t i = 0; i < msg->detections.size(); ++i){
            auto& detection = msg->detections[i]; // Déclarez 'detection' ici pour accéder à l'élément
       	    if (msg->detections[i].id == tag_id_) {
                auto centre = msg->detections[i].centre; // Extraction des coordonnées en pixels
                RCLCPP_INFO(this->get_logger(), "Tag ID: %d, Center in pixels: (u, v) = (%f, %f)",
                            detection.id, centre.x, centre.y);

                // Création et publication d'un message Point
                geometry_msgs::msg::Point point_msg;
                point_msg.x = std::round(centre.x*100)/100;
                point_msg.y = std::round(centre.y*100)/100;
                point_msg.z = 1.0; 
                pixel_coordinates_publisher_->publish(point_msg);
       
       // fin du code ajouté
    }
  }
  
}}
};  // class FindApriltagActionServer

}  // namespace ur_grasping_with_perception

RCLCPP_COMPONENTS_REGISTER_NODE(ur_grasping_with_perception::FindApriltagActionServer)
