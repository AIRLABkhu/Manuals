#include <thread>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Custom action interface - 같은 패키지 내의 action 사용
#include "franka_control_package/action/move_robot.hpp"

using MoveRobotAction = franka_control_package::action::MoveRobot;
using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobotAction>;

class MoveRobot : public rclcpp::Node
{
public:
  MoveRobot() : Node("move_robot")
  {
    RCLCPP_INFO(this->get_logger(), "MoveRobot created");

    // Create action server
    this->action_server_ = rclcpp_action::create_server<MoveRobotAction>(
      this,
      "move_robot_action",
      std::bind(&MoveRobot::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveRobot::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveRobot::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "MoveRobot action server initialized and ready...");

    // Member variables from ROS parameters
    this->declare_parameter("robot_type", "fr3");
    robot_type_ = this->get_parameter("robot_type").as_string();

    if (robot_type_ == "panda") {
      group_name_ = "panda_arm";
      base_link_ = "panda_link0";
    }
    else if (robot_type_ == "fr3") {
      group_name_ = "fr3_arm";
      base_link_ = "fr3_link0";
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "Unknown robot type: %s. using fr3 as default", robot_type_.c_str());
      // use fr3 as default
      group_name_ = "fr3_arm";
      base_link_ = "fr3_link0";
    }
  }

  void initialize_moveit()
  {
    // Create the MoveIt MoveGroup Interface
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), group_name_);

    // Construct and initialize MoveItVisualTools
    moveit_visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        shared_from_this(), base_link_, rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group_interface_->getRobotModel());
    moveit_visual_tools_->deleteAllMarkers();
    moveit_visual_tools_->loadRemoteControl();

    RCLCPP_INFO(this->get_logger(), "MoveIt components initialized");
  }

  ~MoveRobot() = default;

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveRobotAction::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal request with action type: %d", goal->action_type);
    
    if (!move_group_interface_) {
      RCLCPP_ERROR(this->get_logger(), "MoveIt not initialized yet, rejecting goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    
    // Stop the robot if possible
    if (move_group_interface_) {
      move_group_interface_->stop();
    }
    
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
  {
    // Execute the goal in a separate thread
    std::thread{std::bind(&MoveRobot::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveRobotAction::Feedback>();
    auto result = std::make_shared<MoveRobotAction::Result>();

    RCLCPP_INFO(this->get_logger(), "Received target pose: [%.3f, %.3f, %.3f]", 
                goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z);

    // Set the target pose
    move_group_interface_->setPoseTarget(goal->target_pose);

    // Create visualization closures
    auto const draw_title = [this](auto text) {
      if (!moveit_visual_tools_) return;
      auto const text_pose = [] {
        auto msg = Eigen::Isometry3d::Identity();
        msg.translation().z() = 1.0;
        return msg;
      }();
      moveit_visual_tools_->publishText(text_pose, text, rviz_visual_tools::WHITE,
                                       rviz_visual_tools::XLARGE);
    };

    auto const prompt = [this](auto text) {
      moveit_visual_tools_->prompt(text);
    };

    auto const draw_trajectory_tool_path =
        [this, jmg = move_group_interface_->getRobotModel()->getJointModelGroup(
            group_name_)](auto const trajectory) {
          if (!moveit_visual_tools_) return;
          moveit_visual_tools_->publishTrajectoryLine(trajectory, jmg);
        };

    bool planning_success = false;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // Execute based on action type
    switch (goal->action_type) {
      case MoveRobotAction::Goal::PLAN_ONLY:
        {
          RCLCPP_INFO(this->get_logger(), "Executing PLAN_ONLY");
          feedback->current_status = "Planning...";
          goal_handle->publish_feedback(feedback);
          
          draw_title("Planning to target pose");
          if (moveit_visual_tools_) moveit_visual_tools_->trigger();
          
          planning_success = static_cast<bool>(move_group_interface_->plan(plan));
          
          if (planning_success) {
            RCLCPP_INFO(this->get_logger(), "Planning successful!");
            draw_trajectory_tool_path(plan.trajectory_);
            draw_title("Planning Complete");
            result->success = true;
            result->message = "Planning completed successfully";
          } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
            draw_title("Planning Failed!");
            result->success = false;
            result->message = "Planning failed";
          }
          if (moveit_visual_tools_) moveit_visual_tools_->trigger();
        }
        break;

      case MoveRobotAction::Goal::EXECUTE_ONLY:
        {
          RCLCPP_INFO(this->get_logger(), "Executing EXECUTE_ONLY (using last plan)");
          feedback->current_status = "Executing...";
          goal_handle->publish_feedback(feedback);
          
          draw_title("Executing last plan");
          if (moveit_visual_tools_) moveit_visual_tools_->trigger();
          
          auto execute_result = move_group_interface_->move();
          
          if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Execution successful!");
            draw_title("Execution Complete");
            result->success = true;
            result->message = "Execution completed successfully";
          } else {
            RCLCPP_ERROR(this->get_logger(), "Execution failed!");
            draw_title("Execution Failed");
            result->success = false;
            result->message = "Execution failed";
          }
          if (moveit_visual_tools_) moveit_visual_tools_->trigger();
        }
        break;

      case MoveRobotAction::Goal::PLAN_AND_EXECUTE:
      default:
        {
          RCLCPP_INFO(this->get_logger(), "Executing PLAN_AND_EXECUTE");
          
          // Planning phase
          prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
          draw_title("Planning...");

          feedback->current_status = "Planning...";
          goal_handle->publish_feedback(feedback);
          
          if (moveit_visual_tools_) moveit_visual_tools_->trigger();
          
          planning_success = static_cast<bool>(move_group_interface_->plan(plan));
          
          if (planning_success) {
            RCLCPP_INFO(this->get_logger(), "Planning successful! Executing motion...");
            
            draw_trajectory_tool_path(plan.trajectory_);
            if (moveit_visual_tools_) moveit_visual_tools_->trigger();
            
            // Execution phase
            prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
            draw_title("Executing");

            feedback->current_status = "Executing...";
            goal_handle->publish_feedback(feedback);
            
            if (moveit_visual_tools_) moveit_visual_tools_->trigger();
            
            auto execute_result = move_group_interface_->execute(plan);
            
            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
              RCLCPP_INFO(this->get_logger(), "Motion executed successfully!");
              draw_title("Motion Complete");
              result->success = true;
              result->message = "Plan and execute completed successfully";
            } else {
              RCLCPP_ERROR(this->get_logger(), "Motion execution failed!");
              draw_title("Execution Failed");
              result->success = false;
              result->message = "Execution failed";
            }
          } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
            draw_title("Planning Failed!");
            result->success = false;
            result->message = "Planning failed";
          }
          if (moveit_visual_tools_) moveit_visual_tools_->trigger();
        }
        break;
    }

    // Check if goal was cancelled
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Goal was cancelled";
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal cancelled");
      return;
    }

    // Send result
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal completed with result: %s", 
                result->success ? "SUCCESS" : "FAILED");
  }

  std::string robot_type_;
  std::string group_name_;
  std::string base_link_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;
  rclcpp_action::Server<MoveRobotAction>::SharedPtr action_server_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<MoveRobot>();
  
  // Initialize MoveIt components after node creation
  node->initialize_moveit();
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}