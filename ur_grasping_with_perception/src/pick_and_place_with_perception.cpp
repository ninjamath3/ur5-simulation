#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <cmath>
#include "grasping_with_perception_interfaces/action/findapriltag.hpp"


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


//#include "rclcpp_components/register_node_macro.hpp"

//namespace ur_grasping_with_perception
//{


// Structure pour représenter un quaternion
struct Quaternion {
    double x, y, z, w;
};


Quaternion normalize(const Quaternion& q) {
    double magnitude = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return { q.x / magnitude, q.y / magnitude, q.z / magnitude, q.w / magnitude };
}

// Fonction pour faire une rotation autour de l'axe X
Quaternion rotateAroundX(const Quaternion& q, double angleInRadians) {
    Quaternion result;

    // Calcul des composantes du quaternion résultant après rotation autour de l'axe X
    double halfAngle = angleInRadians * 0.5f;
    double sinHalfAngle = sin(halfAngle);
    double cosHalfAngle = cos(halfAngle);

    result.x = q.x * cosHalfAngle + q.w * sinHalfAngle;
    result.y = q.y * cosHalfAngle + q.z * sinHalfAngle;
    result.z = q.z * cosHalfAngle - q.y * sinHalfAngle;
    result.w = q.w * cosHalfAngle - q.x * sinHalfAngle;

    return result;
}  


Quaternion extractRotationAroundZ(const Quaternion& q) {
    // Normaliser le quaternion pour éviter les erreurs d'arrondi
    Quaternion normalizedQuaternion = normalize(q);

    // Extraire l'angle de rotation autour de l'axe Z
    double angleZ = 2.0 * std::acos(normalizedQuaternion.w);

    // Construire un nouveau quaternion aligné selon X et Y avec la rotation autour de Z
    double halfAngleZ = 0.5 * angleZ;
    double sinHalfAngleZ = std::sin(halfAngleZ);
    return { 0.0, 0.0, sinHalfAngleZ, std::cos(halfAngleZ) };
}



static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

class FindApriltagActionClient : public rclcpp::Node
{
public:
  using FindApriltag= grasping_with_perception_interfaces::action::Findapriltag;
  using GoalHandleFindApriltag = rclcpp_action::ClientGoalHandle<FindApriltag>;

  double pos_x, pos_y, orientation_x, orientation_y, orientation_z, orientation_w;
  bool found_object = false;
  explicit FindApriltagActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("findapriltag_action_client", options), goal_done_(false)
  {
    this->client_ptr_ = rclcpp_action::create_client<FindApriltag>(
      this,
      "findapriltag");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FindApriltagActionClient::send_goal, this));
  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();
    this->goal_done_ = false;

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = FindApriltag::Goal();
    goal_msg.tag_id = 333;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<FindApriltag>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FindApriltagActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&FindApriltagActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&FindApriltagActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<FindApriltag>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  void goal_response_callback(const GoalHandleFindApriltag::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleFindApriltag::SharedPtr,
    const std::shared_ptr<const FindApriltag::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    /*for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }*/
    ss << feedback->tracker_status;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleFindApriltag::WrappedResult & result)
  {
    this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    /*for (auto number : result.result->sequence) {
      ss << number << " ";
    }*/
    ss << result.result->tag_found;    
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    found_object = true;
    pos_x = result.result->pose.translation.x;
    pos_y = result.result->pose.translation.y;
    // rotation
    orientation_x = result.result->pose.rotation.x;
    orientation_y = result.result->pose.rotation.y;
    orientation_z = result.result->pose.rotation.z;
    orientation_w = result.result->pose.rotation.w;
    //rclcpp::shutdown();
  }
};  // class FibonacciActionClient



//}  // namespace action_tutorials_cpp

//RCLCPP_COMPONENTS_REGISTER_NODE(ur_grasping_with_perception::FindApriltagActionClient)

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);
  moveit::core::RobotStatePtr current_state_gripper =
      move_group_gripper.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);

  //move_group_arm.setPlanningPipelineId("pilz_industrial_motion_planner");
  //move_group_arm.setPlannerId("PTP");
  
  //move_group_gripper.setPlanningPipelineId("pilz_industrial_motion_planner");
  //move_group_gripper.setPlannerId("PTP");


  move_group_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();
  


  // Go Home
  RCLCPP_INFO(LOGGER, "Going Home");

  // joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
  joint_group_positions_arm[1] = -2.50; // Shoulder Lift
  joint_group_positions_arm[2] = 1.50;  // Elbow
  joint_group_positions_arm[3] = -1.50; // Wrist 1
  joint_group_positions_arm[4] = -1.55; // Wrist 2
  joint_group_positions_arm[5] = 0.00;  // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");
  auto action_client = std::make_shared<FindApriltagActionClient>();
  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }



  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.00;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  if (action_client->found_object) {
    RCLCPP_INFO(LOGGER, " objet trouvé !!");
    //double offsetX = 0.11;
    //double offsetY = -0.015;
    double offsetX = 0.0;
    double offsetY = 0.0;
	  target_pose1.position.x = action_client->pos_x + offsetX;
  	target_pose1.position.y = action_client->pos_y + offsetY;  

    Quaternion initialQuaternion = {action_client->orientation_x, action_client->orientation_y, action_client->orientation_z, action_client->orientation_w};

    // On extrait uniquement la rotation selon Z de ce quaternion, car l'objet est posé à plat.

    Quaternion quatZ = normalize(extractRotationAroundZ(initialQuaternion));

    double rotationAngle = 3.1415;  // Angle de rotation en radians

    // Appel de la fonction pour faire la rotation autour de l'axe X (sinon la pince est orienté vers le haut)
    Quaternion rotatedQuaternion = normalize(rotateAroundX(quatZ, rotationAngle));

    target_pose1.orientation.x = rotatedQuaternion.x;
    target_pose1.orientation.y = rotatedQuaternion.y;
    target_pose1.orientation.z = rotatedQuaternion.z;
    target_pose1.orientation.w = rotatedQuaternion.w;
  }
  else {
    RCLCPP_INFO(LOGGER, " 2 objet non trouvé !!");
  	target_pose1.position.x = 0.343;
  	target_pose1.position.y = 0.132;
  }
  
  target_pose1.position.z = 0.264;
  move_group_arm.setPoseTarget(target_pose1);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Open Gripper

  RCLCPP_INFO(LOGGER, "Open Gripper!");

  move_group_gripper.setNamedTarget("open");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  // Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose1.position.z -= 0.03;
  approach_waypoints.push_back(target_pose1);

  target_pose1.position.z -= 0.03;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group_arm.execute(trajectory_approach);

  // Close Gripper

  RCLCPP_INFO(LOGGER, "Close Gripper!");
  double GP = 0.40;
  joint_group_positions_gripper[0] = GP;
  joint_group_positions_gripper[1] = -GP;
  joint_group_positions_gripper[2] = GP;
  joint_group_positions_gripper[3] = GP;
  joint_group_positions_gripper[4] = -GP;
  joint_group_positions_gripper[5] = GP;
  
  move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
  move_group_gripper.setMaxVelocityScalingFactor(0.001);
  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);
  
  for (int i = 0; i < 30; i++) {
  	GP += 0.005;
  	joint_group_positions_gripper[0] = GP;
  	joint_group_positions_gripper[1] = -GP;
  	joint_group_positions_gripper[2] = GP;
  	joint_group_positions_gripper[3] = GP;
  	joint_group_positions_gripper[4] = -GP;
  	joint_group_positions_gripper[5] = GP;
  
  	move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
  	success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  	move_group_gripper.execute(my_plan_gripper);
 }

  /*move_group_gripper.setNamedTarget("close");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);*/


  // Retreat

  RCLCPP_INFO(LOGGER, "Retreat from object!");

  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  target_pose1.position.z += 0.03;
  retreat_waypoints.push_back(target_pose1);

  target_pose1.position.z += 0.03;
  retreat_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  move_group_arm.execute(trajectory_retreat);

  // Place

  RCLCPP_INFO(LOGGER, "Rotating Arm");

  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  joint_group_positions_arm[0] = 1.57; // Shoulder Pan

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Open Gripper

  RCLCPP_INFO(LOGGER, "Release Object!");

  move_group_gripper.setNamedTarget("open");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  rclcpp::shutdown();
  return 0;

}
