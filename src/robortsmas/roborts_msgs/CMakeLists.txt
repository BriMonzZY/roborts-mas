project(roborts_msgs)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    nav_msgs
    geometry_msgs
    actionlib
    message_generation
    )

add_action_files(
    DIRECTORY action
    FILES
    LocalPlanner.action
    GlobalPlanner.action
    ArmorDetection.action
)

add_message_files(
    DIRECTORY msg
    FILES
    TwistAccel.msg
    GimbalAngle.msg
    GimbalRate.msg
    ObstacleMsg.msg
    ShootInfo.msg
    ShootState.msg
    ArmorPos.msg
    ArmorsPos.msg
    AllyPose.msg
)

add_message_files(
    DIRECTORY msg/referee_system
    FILES
    GameStatus.msg
    GameResult.msg
    GameRobotHP.msg
    GameRobotBullet.msg
    GameZone.msg
    GameZoneArray.msg
    RobotStatus.msg
    RobotHeat.msg
    RobotDamage.msg
    RobotShoot.msg
	LurkStatus.msg
)

add_service_files(
    DIRECTORY srv
    FILES
    GimbalMode.srv
    FricWhl.srv
    ShootCmd.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
)

catkin_package()
