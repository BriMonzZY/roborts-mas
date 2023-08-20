#include <ros/ros.h>

#include "executor/chassis_executor.h"
#include "executor/gimbal_executor.h"

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"

#include <actionlib/client/simple_action_client.h>
#include "roborts_msgs/ArmorDetectionAction.h"
#include <actionlib/client/terminal_state.h>

#include "roborts_msgs/RobotDamage.h"
#include "nav_msgs/Odometry.h"
#include "roborts_msgs/ArmorDetectionActionFeedback.h"

void Command();
char command = '0';

int damage_armor_id;
double odme_angle;
extern bool exit_armor_hurt_state;
extern int state_flag;
extern bool cancel_flag;
extern bool detected_flag;

void HittingReaction(const roborts_msgs::RobotDamage::ConstPtr &robot_hurt)
{
  ROS_ERROR("%d", robot_hurt->damage_source);
  if(robot_hurt->damage_type == 0x0) { // armor hurt
    // cancel_flag = 1;
    command = '3';
    state_flag = 0;
    damage_armor_id = robot_hurt->damage_source;
  }
}

void OdomSub(const nav_msgs::Odometry::ConstPtr &odom)
{
  odme_angle = tf::getYaw(odom->pose.pose.orientation);
  ROS_WARN("odme_angle : %.2f", odme_angle);
}

void DetectionResult(const roborts_msgs::ArmorDetectionActionFeedback::ConstPtr &result)
{
  ROS_ERROR("detected : %d", result->feedback.detected);
  if(result->feedback.detected) detected_flag = 1;
  else detected_flag = 0;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "behavior_test_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto gimbal_executor = new roborts_decision::GimbalExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);

  ros::NodeHandle nh_;
  nh_ = ros::NodeHandle();
  ros::Subscriber ros_robot_hurt_sub_;
  ros::Subscriber ros_odom_sub_;
  ros::Subscriber detection_result_sub_;
  ros_robot_hurt_sub_ = nh_.subscribe("robot_damage", 1, &HittingReaction);  // 订阅机器人状态信息
  ros_odom_sub_ = nh_.subscribe("odom", 1, &OdomSub);
  detection_result_sub_ = nh_.subscribe("/armor_detection_node_action/feedback", 1, &DetectionResult);


  // // armor detection client
  // // create the action client
  // actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> ac("armor_detection_node_action", true);
  // //roborts_msgs::ArmorDetectionResult node_result;
  // ROS_INFO("Waiting for action server to start.");
  // ac.waitForServer();
  // ROS_INFO("Start.");
  // roborts_msgs::ArmorDetectionGoal goal;
  // goal.command = 1;
  // ROS_INFO("I am running the request");
  // ac.sendGoal(goal);



  roborts_decision::BackBootAreaBehavior back_boot_area_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::ChaseBehavior        chase_behavior(gimbal_executor, chassis_executor, blackboard, full_path);
  roborts_decision::SearchBehavior       search_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::EscapeBehavior       escape_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::PatrolBehavior       patrol_behavior(gimbal_executor, chassis_executor, blackboard, full_path);
  roborts_decision::GoalBehavior       goal_behavior(chassis_executor, blackboard);

  auto command_thread= std::thread(Command);
  ros::Rate rate(30);

  // command = '2';

  while(ros::ok()){
    ros::spinOnce();

    ROS_WARN("exit_armor_hurt_state : %d cancel_flag : %d", exit_armor_hurt_state, cancel_flag);
    if(exit_armor_hurt_state) {
      command = '27';
      exit_armor_hurt_state = 0;
    }

    switch (command) {
      //back to boot area 返回启动区
      case '1':
        back_boot_area_behavior.Run();
        break;
        //patrol 定点巡逻
      case '2':
        patrol_behavior.Run();
        break;
        //chase. 追击敌方 jidafanying
      case '3':
        // patrol_behavior.Cancel();
        chase_behavior.Run();
        break;
        //search 在d敌方消失区域搜寻
      case '4':
        search_behavior.Run();
        break;
        //escape. 看到敌方执行逃跑
      case '5':
        escape_behavior.Run();
        break;
        //goal. 指定目标导航
      case '6':
        goal_behavior.Run();
        break;
      case 27:
        if (command_thread.joinable()){
          command_thread.join();
        }
        return 0;
      default:
        break;
    }
    rate.sleep();
  }


  return 0;
}

void Command() {

  while (command != 27) {
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "*********************************please send a command********************************" << std::endl;
    std::cout << "1: back boot area behavior" << std::endl
              << "2: patrol behavior" << std::endl
              << "3: chase_behavior" << std::endl
              << "4: search behavior" << std::endl
              << "5: escape behavior" << std::endl
              << "6: goal behavior" << std::endl
              << "esc: exit program" << std::endl;
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "> ";
    std::cin >> command;
    if (command != '1' && command != '2' && command != '3' && command != '4' && command != '5' && command != '6' && command != 27) {
      std::cout << "please input again!" << std::endl;
      std::cout << "> ";
      std::cin >> command;
    }

  }
}

