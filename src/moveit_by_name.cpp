#include <ros/ros.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_by_name/Command.h>

std::map<std::string, std::unique_ptr<moveit::planning_interface::MoveGroupInterface>> mgi;

void callback(const moveit_by_name::CommandConstPtr& msg){
  if(mgi.count(msg->group) == 0){
    ROS_ERROR_STREAM("received target for unknown joint model group '" << msg->group << "'");
    return;
  }

  if(!mgi[msg->group]->setNamedTarget(msg->target)){
    ROS_ERROR_STREAM("received unknown target '" << msg->target << "' for joint model group '" << msg->group << "'");
    return;
  }

  mgi[msg->group]->move();
  // TODO(v4hn): roswarn on failure
}

int main(int argc, char** argv){
  ros::init(argc, argv, "moveit_by_name");

  ros::NodeHandle nh;

  moveit::core::RobotModelConstPtr robot{ moveit::planning_interface::getSharedRobotModel("robot_description") };

  for(auto& group_name : robot->getJointModelGroupNames()){
    auto group_mgi{ std::make_unique<moveit::planning_interface::MoveGroupInterface>(group_name) };
    group_mgi->setMaxVelocityScalingFactor(1.0);
    group_mgi->setMaxAccelerationScalingFactor(1.0);
    mgi.insert(std::make_pair(group_name, std::move(group_mgi)));
  }

  ros::SubscribeOptions ops;
  ops.template init<moveit_by_name::Command>("moveit_by_name", 10, callback);
  ops.allow_concurrent_callbacks = true;
  ros::Subscriber sub = nh.subscribe(ops);

  // TODO(v4hn): block mgis that are currently in use and leave one spinner free for move responses.
  ros::MultiThreadedSpinner spinner{4};
  spinner.spin();
  ros::waitForShutdown();

  return 0;
}
