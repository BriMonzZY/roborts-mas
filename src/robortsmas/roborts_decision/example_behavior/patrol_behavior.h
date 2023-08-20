#ifndef ROBORTS_DECISION_PATROL_BEHAVIOR_H
#define ROBORTS_DECISION_PATROL_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/TwistAccel.h"

#include "line_iterator.h"

bool cancel_flag = 0;

namespace roborts_decision {
class PatrolBehavior {
 public:
  PatrolBehavior(GimbalExecutor*  &gimbal_executor,
                 ChassisExecutor* &chassis_executor,
                 Blackboard* &blackboard,
                 const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                        gimbal_executor_(gimbal_executor),
                                                        yaw_tmp(0), pitch_tmp(0), yaw_flag(0), pitch_flag(0),
                                                        blackboard_(blackboard) {

    patrol_count_ = 0;
    point_size_ = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {

    auto executor_state = Update();

    std::cout << "state: " << (int)(executor_state) << std::endl;
#if 0
    // 控制底盘
    if (executor_state != BehaviorState::RUNNING) {

      if (patrol_goals_.empty()) {
        ROS_ERROR("patrol goal is empty");
        return;
      }

      std::cout << "send goal" << std::endl;
      chassis_executor_->Execute(patrol_goals_[patrol_count_]);
      patrol_count_ = ++patrol_count_ % point_size_;

    }
    // chassis_executor_->Cancel()
    // if(cancel_flag == 1) {
    //   ROS_ERROR("cancel target");
    //   chassis_executor_->Cancel();
    //   cancel_flag = 0;
    // }
#endif

    // 控制云台
#if 0
    roborts_msgs::GimbalAngle gimbal_angle;
    if (executor_state != BehaviorState::RUNNING) {
      // yaw
      if(yaw_flag == 0 && yaw_tmp < 30.0/180*3.14) {
        yaw_tmp += 2.0/180*3.14;
      }
      else if(yaw_tmp >= 30.0/180*3.14) yaw_flag = 1;
      if(yaw_flag == 1 && yaw_tmp > -30.0/180*3.14) {
        yaw_tmp -= 2.0/180*3.14;
      }
      else if(yaw_tmp <= -30.0/180*3.14) yaw_flag = 0;
      // pitch
      if(pitch_flag == 0 && pitch_tmp < 20.0/180*3.14) {
        pitch_tmp += 1.0/180*3.14;
      }
      else if(pitch_tmp >= 20.0/180*3.14) pitch_flag = 1;
      if(pitch_flag == 1 && pitch_tmp > -0.0/180*3.14) {
        pitch_tmp -= 0.5/180*3.14;
      }
      else if(pitch_tmp <= -0.0/180*3.14) pitch_flag = 0;
      

      gimbal_angle.yaw_mode = 0; // 绝对角度控制
      gimbal_angle.pitch_mode = 0; // 绝对角度控制
      gimbal_angle.yaw_angle = yaw_tmp;
      gimbal_angle.pitch_angle = pitch_tmp;
      gimbal_executor_->Execute(gimbal_angle);
    }
#endif

#if 0
    roborts_msgs::GimbalAngle gimbal_angle;
    geometry_msgs::Twist chassis_speed;

    // yaw
    if(yaw_flag == 0 && yaw_tmp < 20.0/180*3.14) {
      yaw_tmp += 0.001/180*3.14;
    }
    else if(yaw_tmp >= 20.0/180*3.14) yaw_flag = 1;
    if(yaw_flag == 1 && yaw_tmp > -20.0/180*3.14) {
      yaw_tmp -= 0.001/180*3.14;
    }
    else if(yaw_tmp <= -20.0/180*3.14) yaw_flag = 0;

    gimbal_angle.yaw_mode = true;
    gimbal_angle.pitch_mode = 0;
    gimbal_angle.yaw_angle = yaw_tmp;
    ROS_WARN("yaw_tmp : %.2f", yaw_tmp);
    // gimbal_angle.pitch_angle = pitch_tmp;
    gimbal_executor_->Execute(gimbal_angle);

    chassis_speed.linear.x = 0.0;
    chassis_speed.linear.x = 0.0;
    chassis_speed.angular.z = 3.14;
    chassis_executor_->Execute(chassis_speed);
    ROS_WARN("%.2f", chassis_speed.angular.z);

#endif
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    point_size_ = (unsigned int)(decision_config.point().size());
    patrol_goals_.resize(point_size_);
    for (int i = 0; i != point_size_; i++) {
      patrol_goals_[i].header.frame_id = "map";
      patrol_goals_[i].pose.position.x = decision_config.point(i).x();
      patrol_goals_[i].pose.position.y = decision_config.point(i).y();
      patrol_goals_[i].pose.position.z = decision_config.point(i).z();

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.point(i).roll(),
                                                              decision_config.point(i).pitch(),
                                                              decision_config.point(i).yaw());
      patrol_goals_[i].pose.orientation.x = quaternion.x();
      patrol_goals_[i].pose.orientation.y = quaternion.y();
      patrol_goals_[i].pose.orientation.z = quaternion.z();
      patrol_goals_[i].pose.orientation.w = quaternion.w();
    }

    return true;
  }

  ~PatrolBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;
  GimbalExecutor* const gimbal_executor_;

  double yaw_tmp, pitch_tmp;
  bool yaw_flag, pitch_flag;

  //! perception information
  Blackboard* const blackboard_;

  //! patrol buffer
  std::vector<geometry_msgs::PoseStamped> patrol_goals_;
  unsigned int patrol_count_;
  unsigned int point_size_;

};
}

#endif //ROBORTS_DECISION_PATROL_BEHAVIOR_H
