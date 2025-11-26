#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <cmath>
#include <chrono>
#include <thread>
#include "grasping_with_perception_interfaces/action/findapriltag.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"  // Pour abonner au topic joint_states
#include <geometry_msgs/msg/point.hpp>     // publisher de type point
#include <cmath> // Pour std::round
#include "std_msgs/msg/bool.hpp" //pour le flag publisher

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
// Fonction pour faire une rotation autour de l'axe Y 
// <!> fonction ajoutée !! 
Quaternion rotateAroundY(const Quaternion& q, double angleInRadians) {
    Quaternion result;

    // Création du quaternion représentant la rotation autour de l'axe Y
    double halfAngle = angleInRadians * 0.5;
    double sinHalfAngle = sin(halfAngle);
    double cosHalfAngle = cos(halfAngle);

    Quaternion rotation;
    rotation.x = 0.0;
    rotation.y = sinHalfAngle;
    rotation.z = 0.0;
    rotation.w = cosHalfAngle;

    // Multiplication des quaternions : result = rotation * q
    result.x = rotation.w * q.x + rotation.x * q.w + rotation.y * q.z - rotation.z * q.y;
    result.y = rotation.w * q.y - rotation.x * q.z + rotation.y * q.w + rotation.z * q.x;
    result.z = rotation.w * q.z + rotation.x * q.y - rotation.y * q.x + rotation.z * q.w;
    result.w = rotation.w * q.w - rotation.x * q.x - rotation.y * q.y - rotation.z * q.z;

    return result;
}

// Fonction pour faire une rotation autour de l'axe Z
// <!> fonction ajoutée !! 
Quaternion rotateAroundZ(const Quaternion& q, double angleInRadians) {
    Quaternion result;

    // Création du quaternion représentant la rotation autour de l'axe Z
    double halfAngle = angleInRadians * 0.5;
    double sinHalfAngle = sin(halfAngle);
    double cosHalfAngle = cos(halfAngle);

    Quaternion rotation;
    rotation.x = 0.0;
    rotation.y = 0.0;
    rotation.z = sinHalfAngle;
    rotation.w = cosHalfAngle;

    // Multiplication des quaternions : result = rotation * q
    result.x = rotation.w * q.x + rotation.x * q.w + rotation.y * q.z - rotation.z * q.y;
    result.y = rotation.w * q.y - rotation.x * q.z + rotation.y * q.w + rotation.z * q.x;
    result.z = rotation.w * q.z + rotation.x * q.y - rotation.y * q.x + rotation.z * q.w;
    result.w = rotation.w * q.w - rotation.x * q.x - rotation.y * q.y - rotation.z * q.z;

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

/** Classe FindAprilTag pour communiquer avec le serveur d'action et le serveur apriltag**/
class FindApriltagActionClient : public rclcpp::Node
{
public:
  using FindApriltag= grasping_with_perception_interfaces::action::Findapriltag;
  using GoalHandleFindApriltag = rclcpp_action::ClientGoalHandle<FindApriltag>;

  double pos_x, pos_y,pos_z, orientation_x, orientation_y, orientation_z, orientation_w;
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
    pos_z = result.result->pose.translation.z; // <!> ligne ajoutée !!
    // rotation
    orientation_x = result.result->pose.rotation.x;
    orientation_y = result.result->pose.rotation.y;
    orientation_z = result.result->pose.rotation.z;
    orientation_w = result.result->pose.rotation.w;
    //rclcpp::shutdown();
  }
};

/*-----------------------fonctions ajoutées-----------------------*/

// Fonction pour traiter les messages /joint_states
void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Afficher les positions des joints
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Received joint states:");
    for (size_t i = 0; i < msg->name.size(); ++i) {
        RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), 
                    "Joint: %s, Position: %f", msg->name[i].c_str(), msg->position[i]);
    }
    
    // Vérifier le timestamp
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), 
                "Timestamp: sec=%d, nanosec=%d", msg->header.stamp.sec, msg->header.stamp.nanosec);
}

/* Fonction pour ouvrir le gripper */
void open_gripper(moveit::planning_interface::MoveGroupInterface& move_group_gripper) {
    if (move_group_gripper.getJointModelGroupNames().empty()) { 
        RCLCPP_ERROR(rclcpp::get_logger("move_group_interface_tutorial"), "Gripper joint model group not initialized.");
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Opening Gripper...");
    move_group_gripper.setNamedTarget("open");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
    bool success_gripper = (move_group_gripper.plan(my_plan_gripper) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success_gripper) {
        move_group_gripper.execute(my_plan_gripper);
        RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Gripper opened successfully.");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("move_group_interface_tutorial"), "Failed to plan for gripper.");
    }
}

/* Fonction pour fermer le gripper tout doucement*/

void close_gripper(std::vector<double>joint_group_positions_gripper,moveit::planning_interface::MoveGroupInterface& move_group_gripper){


    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Close Gripper!");
    double GP = 0.305; //40 <!>
    joint_group_positions_gripper[0] = GP;
    joint_group_positions_gripper[1] = -GP;
    joint_group_positions_gripper[2] = GP;
    joint_group_positions_gripper[3] = GP;
    joint_group_positions_gripper[4] = -GP;
    joint_group_positions_gripper[5] = GP;
  
    move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
    move_group_gripper.setMaxVelocityScalingFactor(0.001);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
    
    bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success_gripper){
        move_group_gripper.execute(my_plan_gripper);
    }
    else {
    	RCLCPP_ERROR(rclcpp::get_logger("move_group_interface_tutorial"), "Failed to plan for gripper.");
    }
  
    for (int i = 0; i < 10; i++) {
  	GP += 0.022;
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
}


/* Fonction pour initialiser le robot */
void robot_initialization(int argc, char** argv, 
                          std::shared_ptr<rclcpp::Node>& move_group_node, 
                          moveit::planning_interface::MoveGroupInterface*& move_group_arm, 
                          moveit::planning_interface::MoveGroupInterface*& move_group_gripper) {

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides({{"use_sim_time", true}});
    move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    // Abonnement au topic /joint_states
    auto joint_state_subscriber = move_group_node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, joint_state_callback);

    auto clock = rclcpp::Clock(RCL_SYSTEM_TIME);
    auto start_time = clock.now();
    while (rclcpp::ok()) {
        if (rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds() > 0) {
            RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Simulation clock synchronized.");
            break;
        }
        if ((clock.now() - start_time).seconds() > 10.0) {
            RCLCPP_ERROR(rclcpp::get_logger("move_group_interface_tutorial"), "Timeout waiting for simulation clock.");
            throw std::runtime_error("Simulation clock not synchronized.");
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
    static const std::string PLANNING_GROUP_GRIPPER = "gripper";

    move_group_arm = new moveit::planning_interface::MoveGroupInterface(move_group_node, PLANNING_GROUP_ARM);
    move_group_gripper = new moveit::planning_interface::MoveGroupInterface(move_group_node, PLANNING_GROUP_GRIPPER);

    open_gripper(*move_group_gripper);
}

/* Fonction pour tourner les joints du robot 
   A executer après être retourner à la positionde référence {0.0, -2.50, 1.50, -1.50, -1.55, 0.0} (en rad)
   Créer une nouvelle cible à atteindre 

*/
void reach_target(moveit::planning_interface::MoveGroupInterface& move_group_arm, std::vector<double> new_joint_positions ) {
    std::vector<double> joint_group_positions_arm = {0.0+new_joint_positions[0], -2.50+new_joint_positions[1], 1.50+new_joint_positions[2], -1.50+new_joint_positions[3], -1.55+new_joint_positions[4], 0.0+new_joint_positions[5]};
    move_group_arm.setJointValueTarget(joint_group_positions_arm);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    bool success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success_arm) {
        RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "reaching target ...");
        move_group_arm.execute(my_plan_arm);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("move_group_interface_tutorial"), "Failed to reach target !");
    }
}

/* Fonction pour introduire une pause */
void delay_ms(int milliseconds) {
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "-------Début de la pause--------");
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "-------Fin de la pause--------");
}

/* Aller à la position de référence */
void go_home(moveit::planning_interface::MoveGroupInterface& move_group_arm) {
    std::vector<double> joint_group_positions_arm = {0.0, -2.50, 1.50, -1.50, -1.55, 0.0};
    move_group_arm.setJointValueTarget(joint_group_positions_arm);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    bool success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success_arm) {
        RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Going Home");
        move_group_arm.execute(my_plan_arm);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("move_group_interface_tutorial"), "Failed to plan for arm.");
    }
}


/* Positionne la pince sur le côté du cube et bien orientée pour pouvoir le saisir*/

geometry_msgs::msg::Pose pre_grasp(moveit::planning_interface::MoveGroupInterface& move_group_arm){
  RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Pregrasp Position");
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
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), " objet trouvé !!");
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
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), " 2 objet non trouvé !!");
  	target_pose1.position.x = 0.343;
  	target_pose1.position.y = 0.132;
  }
  
  target_pose1.position.z = 0.264;

  double rotationAngle = 1.5708;  // 90 degrés en radians
  
  Quaternion initialQuaternion = {target_pose1.orientation.x, 
                                    target_pose1.orientation.y, 
                                    target_pose1.orientation.z, 
                                    target_pose1.orientation.w};
     
  Quaternion rotatedQuaternion2 = normalize(rotateAroundX(initialQuaternion, rotationAngle));

  target_pose1.orientation.x = rotatedQuaternion2.x;
  target_pose1.orientation.y = rotatedQuaternion2.y;
  target_pose1.orientation.z = rotatedQuaternion2.z;
  target_pose1.orientation.w = rotatedQuaternion2.w;
  
   
  // Déplacer la position du robot pour qu'il puisse agripper le cube par le côté
  double offset_x = 0.0;  // Décalage latéral par rapport à la pose préhenseur pour l'approche latérale
  double offset_y = -0.1;  
  double offset_z = 0.0;  

  target_pose1.position.x += offset_x;
  target_pose1.position.y += offset_y;
  target_pose1.position.z += offset_z;
  
  Quaternion rotatedQuaternion3 = normalize(rotateAroundX(rotatedQuaternion2, 2.0*rotationAngle));
  
  target_pose1.orientation.x = rotatedQuaternion3.x;
  target_pose1.orientation.y = rotatedQuaternion3.y;
  target_pose1.orientation.z = rotatedQuaternion3.z;
  target_pose1.orientation.w = rotatedQuaternion3.w;
  
  // Déplacer la position du robot pour qu'il puisse agripper le cube par le côté
  double offset_x2 = 0.0;  // Décalage latéral par rapport à la pose préhenseur pour l'approche latérale
  double offset_y2 = 0.25;  
  double offset_z2 = 0.0;  

  target_pose1.position.x += offset_x2;
  target_pose1.position.y += offset_y2;
  target_pose1.position.z += offset_z2;
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;//
  move_group_arm.setPoseTarget(target_pose1);

  bool success_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success_arm) {
        RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Pregrasp !! ");
        move_group_arm.execute(my_plan_arm);
  } 
    
  else {
        RCLCPP_ERROR(rclcpp::get_logger("move_group_interface_tutorial"), "Failed to plan for arm.");
  }
  
  return target_pose1;
}


/*Fonction qui permet de s'approcher du cube si le gripper est bien positionné*/

void approach_cube(geometry_msgs::msg::Pose target_pose1, moveit::planning_interface::MoveGroupInterface& move_group_arm){
  RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Approach to object!");

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose1.position.z -= 0.21;
  approach_waypoints.push_back(target_pose1);

  target_pose1.position.z -= 0.03;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  move_group_arm.computeCartesianPath(approach_waypoints, eef_step, jump_threshold, trajectory_approach);


  move_group_arm.execute(trajectory_approach);
}


/*Fonction qui permet d'éloigner le cube du sol quand il est aggripé*/
void retreat_cube(geometry_msgs::msg::Pose target_pose1, moveit::planning_interface::MoveGroupInterface& move_group_arm){
  RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Retreat from object!");

  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  target_pose1.position.z += 0.21;
  retreat_waypoints.push_back(target_pose1);

  target_pose1.position.z += 0.03;
  retreat_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat;
  
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  
   move_group_arm.computeCartesianPath(retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  move_group_arm.execute(trajectory_retreat);
}

/* ------------- Fonction main ----------------- */
int main(int argc, char** argv) {
    std::shared_ptr<rclcpp::Node> move_group_node;
    moveit::planning_interface::MoveGroupInterface* move_group_arm;
    moveit::planning_interface::MoveGroupInterface* move_group_gripper;
    
    std::vector<double> joint_group_positions_gripper(6, 0.0);

    robot_initialization(argc, argv, move_group_node, move_group_arm, move_group_gripper);

    // Créer des noeuds (1 pour les points 3D et 1 autre pour signaler quand le robot à fini de bouger)
    auto node = rclcpp::Node::make_shared("points_mathys_3D");
    auto node2 =rclcpp::Node::make_shared("flag_publisher");

    // Créer des publishers...
    auto publisher = node->create_publisher<geometry_msgs::msg::Point>("point_topic_mathys_3D", 10);
    auto publisher2 = node->create_publisher<std_msgs::msg::Bool>("/flag_topic", 10);
    
    // ...de type Point
    auto point = geometry_msgs::msg::Point();
    
    // ... et de type Bool
    std_msgs::msg::Bool msg;
    msg.data=false;
    publisher2->publish(msg);

    // Pause de 1 seconde
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Pause de 1s");
    delay_ms(1000);
   
    
    // Aller à la position de référence
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Vers la position de référence");
    go_home(*move_group_arm);
    
    // Pause de 1 seconde
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Pause de 1s");
    delay_ms(1000);
    
    // Positionnement du robot juste au-dessus du cube et récupération de la target
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Lancement de pregrasp ....");
    geometry_msgs::msg::Pose target_pose;
    target_pose=pre_grasp(*move_group_arm); //recupération de target_pose
    
    // Pause de 1 seconde
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Pause de 1s");
    delay_ms(1000);
    
    //Ouverture du gripper 
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Ouverture du Gripper");
    open_gripper(* move_group_gripper);
    
    // Pause de 1 seconde
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Pause de 1s");
    delay_ms(1000);
    
    // Approche du robot pour saisir le cube 
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Approche du cube ");
    approach_cube(target_pose, *move_group_arm);

    // Pause de 1 seconde
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Pause de 1s");
    delay_ms(1000);
    
    // Fermeture du gripper
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Fermeture lente du Gripper");
    close_gripper(joint_group_positions_gripper,* move_group_gripper);
    
    // Pause de 1 seconde
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Pause de 1s");
    delay_ms(1000);
    
    // Eloignement du cube par rapport au sol
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Eloignement du cube par rapport au sol ");
    retreat_cube(target_pose, *move_group_arm);
    
    //Pause de 500ms
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Pause de 500ms");
    delay_ms(500);

    const double eef_step = 0.005; 
    const double jump_threshold = 0.0; 
    double fraction;
		   
    
    int x_end = 3;
    int y_end = 2;
    int z_end = 3;
    

    moveit_msgs::msg::RobotTrajectory trajectory_myre;
    std::vector<geometry_msgs::msg::Pose> myre_deplacement;
  
    for (int xi = -3; xi < x_end; xi++) {
        for (int yi = 1; yi < y_end; yi++) {
             for (int zi = 1; zi<z_end; zi++){
        
		    if (xi==0 || yi==0 || zi==0){
		        publisher2->publish(msg);
		        RCLCPP_WARN(rclcpp::get_logger("move_group_interface_tutorial"), "Flag vaut False ");
		    	continue;
		    }
		    
		    // Pause de 1 seconde
	    	    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Pause de 1s");
		    delay_ms(1000);
		    
		    // Réinitialisation 
		    myre_deplacement.clear(); // Évite l'accumulation des anciennes poses
		    
    		    move_group_arm->setPlanningTime(15.0);  // Augmente le temps de calcul pour un meilleur résultat
    		    move_group_arm->setNumPlanningAttempts(20);  // Essaye plusieurs plans et prend le meilleur

		    
		    // Mise à jour des positions
		    target_pose.position.x = std::round(xi * 0.075*100)/100; 
		    target_pose.position.y = std::round(yi * 0.075*100)/100; 
		    target_pose.position.z = std::round(zi * 0.025*100)/100; 
		    

		    // Log des positions
		    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"),
		            "Iteration (xi=%d, yi=%d, zi=%d): Position x=%.2f, y=%.2f, z=%.2f",
		            xi, yi, zi, target_pose.position.x, target_pose.position.y,target_pose.position.z);
		            
		    // mise à jour de la vitesse
                    move_group_arm->setMaxVelocityScalingFactor(0.5); // 50% de la vitesse maximale
                    move_group_arm->setMaxAccelerationScalingFactor(0.5); //50% de l'accélération maximale
		                        
		    myre_deplacement.push_back(target_pose);


                    fraction = move_group_arm->computeCartesianPath(myre_deplacement, eef_step, jump_threshold, trajectory_myre);

		    // Vérification du succès du chemin
	 	    if (fraction == 0.0) { // Si aucune trajectoire n'est trouvée
				RCLCPP_WARN(rclcpp::get_logger("move_group_interface_tutorial"), "Path planning failed");
			        point.x = 0;
    		                point.y = 0;
    		                point.z = 0;
    		    
                                publisher->publish(point);

				if (xi == x_end - 1 && yi == y_end - 1 && zi == z_end - 1) {
				    RCLCPP_WARN(rclcpp::get_logger("move_group_interface_tutorial"), "Boucle finie ! ");
				    msg.data=true;
                                    publisher2->publish(msg);
                                    RCLCPP_WARN(rclcpp::get_logger("move_group_interface_tutorial"), "Flag vaut True ");
				}
				else{
				    publisher2->publish(msg);
				    RCLCPP_WARN(rclcpp::get_logger("move_group_interface_tutorial"), "Flag vaut False ");
				}
				continue;
			    } 
		    else {
		 		
			RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"),
			"Path planned successfully with fraction=%.2f", fraction);
	             }


		    // Exécuter la trajectoire
		    move_group_arm->execute(trajectory_myre);
		    
		    // Pause de 1 seconde
	    	    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Pause de 1s");
		    delay_ms(1000);
		    
		    // Publication du message 
		    point.x = target_pose.position.x;
    		    point.y = target_pose.position.y;
    		    point.z = target_pose.position.z;
    		    
    		    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Publishing: x=%.2f, y=%.2f, z=%.2f", point.x, point.y, point.z);
    		    
                    publisher->publish(point);
		     
		    // Pause de 3 seconde
	    	    RCLCPP_INFO(rclcpp::get_logger("move_group_interface_tutorial"), "Pause de 3s");
		    delay_ms(3000);
		    
		    // On publie le flag
		    
		    if (xi == x_end - 1 && yi == y_end - 1 && zi == z_end - 1) {
			RCLCPP_WARN(rclcpp::get_logger("move_group_interface_tutorial"), "Boucle finie ! ");
			msg.data=true;
                        publisher2->publish(msg);
                        RCLCPP_WARN(rclcpp::get_logger("move_group_interface_tutorial"), "Flag vaut True ");
		    }
		    else{
                        RCLCPP_WARN(rclcpp::get_logger("move_group_interface_tutorial"), "Flag vaut False ");
		    }

           }
        }
    }


    
    rclcpp::shutdown();
    return 0;
}





