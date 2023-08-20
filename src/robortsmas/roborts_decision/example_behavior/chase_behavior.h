#ifndef ROBORTS_DECISION_CHASE_BEHAVIOR_H
#define ROBORTS_DECISION_CHASE_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

extern int damage_armor_id;
extern double odme_angle;
double first_angle;
int state_flag = 0;
ros::Time time_first;
bool exit_armor_hurt_state = 0;
bool detected_flag = 0;

namespace roborts_decision {
class ChaseBehavior {
 public:
  ChaseBehavior(GimbalExecutor*  &gimbal_executor,
                ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       gimbal_executor_(gimbal_executor),
                                                       blackboard_(blackboard) {

#if 0
    chase_goal_.header.frame_id = "map";
    chase_goal_.pose.orientation.x = 0;
    chase_goal_.pose.orientation.y = 0;
    chase_goal_.pose.orientation.z = 0;
    chase_goal_.pose.orientation.w = 1;

    chase_goal_.pose.position.x = 0;
    chase_goal_.pose.position.y = 0;
    chase_goal_.pose.position.z = 0;

    chase_buffer_.resize(2);
    chase_count_ = 0;

    cancel_goal_ = true;
#endif
  }

  void Run() {

    // auto executor_state = Update();
    // std::cout << "state: " << (int)(executor_state) << std::endl;
#if 0
    auto robot_map_pose = blackboard_->GetRobotMapPose();
    if (executor_state != BehaviorState::RUNNING) {

      chase_buffer_[chase_count_++ % 2] = blackboard_->GetEnemy();

      chase_count_ = chase_count_ % 2;

      auto dx = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.x - robot_map_pose.pose.position.x;
      auto dy = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.y - robot_map_pose.pose.position.y;
      auto yaw = std::atan2(dy, dx);

      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) >= 1.0 && std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2.0) {
        if (cancel_goal_) {
          chassis_executor_->Cancel();
          cancel_goal_ = false;
        }
        return;

      } else {

        auto orientation = tf::createQuaternionMsgFromYaw(yaw);
        geometry_msgs::PoseStamped reduce_goal;
        reduce_goal.pose.orientation = robot_map_pose.pose.orientation;

        reduce_goal.header.frame_id = "map";
        reduce_goal.header.stamp = ros::Time::now();
        reduce_goal.pose.position.x = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.x - 1.2 * cos(yaw);
        reduce_goal.pose.position.y = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.y - 1.2 * sin(yaw);
        auto enemy_x = reduce_goal.pose.position.x;
        auto enemy_y = reduce_goal.pose.position.y;
        reduce_goal.pose.position.z = 1;
        unsigned int goal_cell_x, goal_cell_y;

        // if necessary add mutex lock
        //blackboard_->GetCostMap2D()->GetMutex()->lock();
        auto get_enemy_cell = blackboard_->GetCostMap2D()->World2Map(enemy_x,
                                                                   enemy_y,
                                                                   goal_cell_x,
                                                                   goal_cell_y);
        //blackboard_->GetCostMap2D()->GetMutex()->unlock();

        if (!get_enemy_cell) {
          return;
        }

        auto robot_x = robot_map_pose.pose.position.x;
        auto robot_y = robot_map_pose.pose.position.y;
        unsigned int robot_cell_x, robot_cell_y;
        double goal_x, goal_y;
        blackboard_->GetCostMap2D()->World2Map(robot_x,
                                              robot_y,
                                              robot_cell_x,
                                              robot_cell_y);

        if (blackboard_->GetCostMap2D()->GetCost(goal_cell_x, goal_cell_y) >= 253) {

          bool find_goal = false;
          for(FastLineIterator line( goal_cell_x, goal_cell_y, robot_cell_x, robot_cell_x); line.IsValid(); line.Advance()) {

            auto point_cost = blackboard_->GetCostMap2D()->GetCost((unsigned int) (line.GetX()), (unsigned int) (line.GetY())); //current point's cost

            if(point_cost >= 253){
              continue;

            } else {
              find_goal = true;
              blackboard_->GetCostMap2D()->Map2World((unsigned int) (line.GetX()),
                                                     (unsigned int) (line.GetY()),
                                                     goal_x,
                                                     goal_y);

              reduce_goal.pose.position.x = goal_x;
              reduce_goal.pose.position.y = goal_y;
              break;
            }

          }
          if (find_goal) {
            cancel_goal_ = true;
            chassis_executor_->Execute(reduce_goal);
          } else {
            if (cancel_goal_) {
              chassis_executor_->Cancel();
              cancel_goal_ = false;
            }
            return;
          }

        } else {
          cancel_goal_ = true;
          chassis_executor_->Execute(reduce_goal);
        }
      }
    }
#endif

#if 0
    auto executor_state = Update();
    // std::cout << "state: " << (int)(executor_state) << std::endl;
    std::cout << "armor hurt state: " << (int)(state_flag) << std::endl;

    roborts_msgs::GimbalAngle gimbal_angle;
    geometry_msgs::Twist chassis_speed;

    ROS_WARN("%.2f  %.2f", fabs(odme_angle-first_angle), fabs(fabs(odme_angle-first_angle)-180.0/180*3.14));
    if(state_flag == 1 && (
      damage_armor_id == 0 ||
      damage_armor_id == 1 && fabs(fabs(odme_angle-first_angle)-90.0/180*3.14) <= 20.0/180*3.14 ||
      damage_armor_id == 2 && fabs(fabs(odme_angle-first_angle)-180.0/180*3.14) <= 20.0/180*3.14 ||
      damage_armor_id == 3 && fabs(fabs(first_angle-odme_angle)-90.0/180*3.14) <= 20.0/180*3.14)
      ) {
      state_flag = 2;
    }

    if (state_flag == 0) {// if (executor_state != BehaviorState::RUNNING && once_flag==0) {
      state_flag = 1;
      first_angle = odme_angle;
      time_first = ros::Time::now();
      if(damage_armor_id == 0) {
        gimbal_angle.yaw_mode = true;
        gimbal_angle.pitch_mode = 0;
        gimbal_angle.yaw_angle = 0.0/180*3.1415;  // front
      }
      if(damage_armor_id == 1) {
        gimbal_angle.yaw_mode = true;
        gimbal_angle.pitch_mode = 0;
        gimbal_angle.yaw_angle = 85.0/180*3.1415;  // left
      }
      else if(damage_armor_id == 2) {
        gimbal_angle.yaw_mode = true;
        gimbal_angle.pitch_mode = 0;
        gimbal_angle.yaw_angle = 160.0/180*3.1415;  // back
      }
      else if(damage_armor_id == 3) {
        gimbal_angle.yaw_mode = true;
        gimbal_angle.pitch_mode = 0;
        gimbal_angle.yaw_angle = -85.0/180*3.1415;  // back
      }
      gimbal_executor_->Execute(gimbal_angle);
    }
    else if(state_flag == 2) { 
      gimbal_angle.yaw_mode = true;
      gimbal_angle.pitch_mode = 0;
      gimbal_angle.yaw_angle = 0.0;
      // gimbal_angle.pitch_angle = pitch_tmp;
      gimbal_executor_->Execute(gimbal_angle);

      chassis_speed.linear.x = 0.0;
      chassis_speed.linear.x = 0.0;
      chassis_speed.angular.z = 7;
      chassis_executor_->Execute(chassis_speed);
      // ROS_WARN("%.2f", chassis_speed.angular.z); 
      gimbal_angle.yaw_mode = false;
      gimbal_angle.pitch_mode = true;
      gimbal_angle.yaw_angle = 0.0;
      gimbal_executor_->Execute(gimbal_angle);

      ros::Time time = ros::Time::now();
      if((time - time_first).toSec() >= 4) {
        state_flag = 4;
      }
    }
    else if(state_flag == 4) {
      exit_armor_hurt_state = 1;
    }

#endif
// absolute 4-8-18-55
#if 1
    auto executor_state = Update();
    std::cout << "armor hurt state: " << (int)(state_flag) << std::endl;

    roborts_msgs::GimbalAngle gimbal_angle;
    geometry_msgs::Twist chassis_speed;

    if(state_flag == 0) {
      state_flag = 1;
      time_first = ros::Time::now();
    }
    if (state_flag == 1) {  // 开始小陀螺
      // if(detected_flag == false) {
      //   gimbal_angle.yaw_mode = 1;
      //   gimbal_angle.pitch_mode = 0;
      //   gimbal_angle.yaw_angle = 6.0/180*3.1415;
      //   gimbal_executor_->Execute(gimbal_angle);
      // }
      
      chassis_speed.linear.x = 0.0;
      chassis_speed.linear.x = 0.0;
      chassis_speed.angular.z = -5.5;  // 6 小陀螺旋转速度
      chassis_executor_->Execute(chassis_speed);

      ros::Time time = ros::Time::now();
      if((time - time_first).toSec() >= 6) {  // 小陀螺旋转时间
        time_first = ros::Time::now();
        state_flag = 2;
      }
    }
    else if(state_flag == 2) {
      // if(detected_flag == false) {
      //   gimbal_angle.yaw_mode = 0;
      //   gimbal_angle.pitch_mode = 0;
      //   gimbal_angle.yaw_angle = 0.0;
      //   gimbal_executor_->Execute(gimbal_angle);
      // }
      ros::Time time = ros::Time::now();
      if((time - time_first).toSec() >= 1) {
        state_flag = 3;
        exit_armor_hurt_state = 1;
      }
    }
    // else if(state_flag == 2) { 
    //   gimbal_angle.yaw_mode = 0;
    //   gimbal_angle.pitch_moe = 0;
    //   gimbal_angle.yaw_angle = 0.0;
    //   gimbal_executor_->Execute(gimbal_angle);
    // }
#endif
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    // return chassis_executor_->Update();
    return gimbal_executor_->Update();
  }

  void SetGoal(geometry_msgs::PoseStamped chase_goal) {
    chase_goal_ = chase_goal;
  }

  ~ChaseBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;
  GimbalExecutor* const gimbal_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! chase goal
  geometry_msgs::PoseStamped chase_goal_;

  //! chase buffer
  std::vector<geometry_msgs::PoseStamped> chase_buffer_;
  unsigned int chase_count_;

  //! cancel flag
  bool cancel_goal_;
};
}

#endif //ROBORTS_DECISION_CHASE_BEHAVIOR_H
