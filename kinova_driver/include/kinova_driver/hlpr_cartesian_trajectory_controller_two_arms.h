#ifndef JACO_CARTESIAN_TRAJECTORY_CONTROLLER_H_
#define JACO_CARTESIAN_TRAJECTORY_CONTROLLER_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ecl/geometry.hpp>
#include <std_srvs/SetBool.h>
#include <kinova_msgs/PoseVelocity.h>


#define NUM_DOFS 6

#define LINEAR_VELOCITY 0.8378 //maximum linear velocity (x,y,z) (m/s)
#define ANGULAR_VELOCITY 1.0472 //maximum angular velocity (r,p,y)) (rad/s)
#define TIME_SCALING_FACTOR 1.75 //keep the trajectory at a followable speed

#define DEG_TO_RAD (M_PI/180)
#define RAD_TO_DEG (180/M_PI)

//gains for trajectory follower
#define KP 225.0
#define KV 10.0
#define ERROR_THRESHOLD .03 //threshold in radians for combined joint error to consider motion a success

class JacoCartesianTrajectoryController
{
public:
  JacoCartesianTrajectoryController();

  bool useTimeService(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res);

  void executeSmoothTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

private:
  ros::NodeHandle n;
  ros::NodeHandle pnh;

  //TF
  tf::TransformListener tf_listener;

  //Actions
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> smoothTrajectoryServer;

  // Messages
  ros::Publisher cartesianCmdPublisher;

  // Fake gravity comp services for simulation
  ros::ServiceServer use_time_service_;

  // Parameters
  double maxCurvature;
  bool   use_time_flag_;
  std::string side_;

  std::vector<std::string> jointNames;
};

#endif

