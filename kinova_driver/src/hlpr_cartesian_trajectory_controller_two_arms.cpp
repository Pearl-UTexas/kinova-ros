#include <kinova_driver/hlpr_cartesian_trajectory_controller_two_arms.h>

using namespace std;

// Initialization
JacoCartesianTrajectoryController::JacoCartesianTrajectoryController() : pnh("~"), smoothTrajectoryServer(pnh, "trajectory", boost::bind(&JacoCartesianTrajectoryController::executeSmoothTrajectory, this, _1), false)
{
  pnh.param("max_curvature", maxCurvature, 200.0);
  pnh.param<string>("side", side_, "right");
  pnh.param("use_time", use_time_flag_, false);

  jointNames.clear();
  jointNames.push_back(side_ + "_eef_x");
  jointNames.push_back(side_ + "_eef_y");
  jointNames.push_back(side_ + "_eef_z");
  jointNames.push_back(side_ + "_eef_roll");
  jointNames.push_back(side_ + "_eef_pitch");
  jointNames.push_back(side_ + "_eef_yaw");

  ROS_INFO("Using real robot arm.");

  // Connect to the low-level angular driver from kinova-ros
  cartesianCmdPublisher = n.advertise<kinova_msgs::PoseVelocity>(side_+"_arm_driver/in/cartesian_velocity", 1);

  // Setup use time service
  use_time_service_ = n.advertiseService("use_custom_time", &JacoCartesianTrajectoryController::useTimeService, this);

  // Start the trajectory server
  smoothTrajectoryServer.start();
}


bool JacoCartesianTrajectoryController::useTimeService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    ROS_INFO("Toggling use time");
    use_time_flag_ = req.data;
    return true;
}

/** Adjust angle to equivalent angle on [-pi, pi]
 *  @param angle the angle to be simplified (-inf, inf)
 *  @return the simplified angle on [-pi, pi]
 */
static inline double simplify_angle(double angle)
{
  double previous_rev = floor(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double next_rev = ceil(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double current_rev;
  if (fabs(angle - previous_rev) < fabs(angle - next_rev))
    return angle - previous_rev;
  return angle - next_rev;
}



/** Calculates nearest desired angle to the current angle
 *  @param desired desired joint angle [-pi, pi]
 *  @param current current angle (-inf, inf)
 *  @return the closest equivalent angle (-inf, inf)
 */
static inline double nearest_equivalent_angle(double desired, double current)
{
  //calculate current number of revolutions
  double previous_rev = floor(current / (2 * M_PI));
  double next_rev = ceil(current / (2 * M_PI));
  double current_rev;
  if (fabs(current - previous_rev * 2 * M_PI) < fabs(current - next_rev * 2 * M_PI))
    current_rev = previous_rev;
  else
    current_rev = next_rev;

  //determine closest angle
  double lowVal = (current_rev - 1) * 2 * M_PI + desired;
  double medVal = current_rev * 2 * M_PI + desired;
  double highVal = (current_rev + 1) * 2 * M_PI + desired;
  if (fabs(current - lowVal) <= fabs(current - medVal) && fabs(current - lowVal) <= fabs(current - highVal))
    return lowVal;
  if (fabs(current - medVal) <= fabs(current - lowVal) && fabs(current - medVal) <= fabs(current - highVal))
    return medVal;
  return highVal;
}


/* execute Smooth Cartesian Controller
// get trajectory data
// Initialize array to fit a smooth trajectory controller
// determine time compenent
// Calculate Spline for each dof
// Control loop --> Calculate the error based velocity commands
*/

void JacoCartesianTrajectoryController::executeSmoothTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
  float trajectoryPoints[NUM_DOFS][goal->trajectory.points.size()];
  int numPoints = goal->trajectory.points.size();

  //get trajectory data
  for (unsigned int i = 0; i < numPoints; i++)
  {
    for (int trajectoryIndex = 0; trajectoryIndex < goal->trajectory.joint_names.size(); trajectoryIndex ++)
    {
      string jointName = goal->trajectory.joint_names[trajectoryIndex];
      int jointIndex = distance(jointNames.begin(), find(jointNames.begin(), jointNames.end(), jointName));
      if (jointIndex >= 0 && jointIndex < NUM_DOFS)
      {
        trajectoryPoints[jointIndex][i] = goal->trajectory.points.at(i).positions.at(trajectoryIndex);
      }
    }
  }

  //initialize arrays needed to fit a smooth trajectory to the given points
  ecl::Array<double> timePoints(numPoints);
  timePoints[0] = 0.0;
  vector<ecl::Array<double> > cartPoints;
  cartPoints.resize(NUM_DOFS);
  float prevPoint[NUM_DOFS];
  for (unsigned int i = 0; i < NUM_DOFS; i++)
  {
    cartPoints[i].resize(numPoints);
    cartPoints[i][0] = trajectoryPoints[i][0];
    prevPoint[i] = trajectoryPoints[i][0];
  }

  //determine time component of trajectories for each joint
  if (use_time_flag_)
  {
    for (unsigned int i = 1; i < numPoints; i++)
      timePoints[i] = goal->trajectory.points.at(i).time_from_start.toSec();
  }
  else
  {
    for (unsigned int i = 1; i < numPoints; i++)
    {
      float maxTime = 0.0;
      for (unsigned int j = 0; j < NUM_DOFS; j++)
      {
        //calculate approximate time required to move to the next position
        float time = fabs(trajectoryPoints[j][i] - prevPoint[j]);
        if (j <= 2)
          time /= LINEAR_VELOCITY;
        else
          time /= ANGULAR_VELOCITY;

        if (time > maxTime)
          maxTime = time;

        cartPoints[j][i] = trajectoryPoints[j][i];
        prevPoint[j] = trajectoryPoints[j][i];
      }

      timePoints[i] = timePoints[i - 1] + maxTime * TIME_SCALING_FACTOR;
    }
  }

  vector<ecl::SmoothLinearSpline> splines;
  splines.resize(NUM_DOFS);

  // Catch error in max bound error
  try{
    for (unsigned int i = 0; i < NUM_DOFS; i++)
    {
      ecl::SmoothLinearSpline tempSpline(timePoints, cartPoints[i], maxCurvature);
      splines.at(i) = tempSpline;
    }
  } 
  catch ( std::exception &exc ){ 
      std::cerr << exc.what();
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      smoothTrajectoryServer.setAborted(result);
      ROS_ERROR("Trajectory could not be generated. Aborting trajectory action.");

      //not_safe_for_gc_ = false; // Can go into kinesthetic mode again
    return;
  }


  //control loop
  bool trajectoryComplete = false;
  double startTime = ros::Time::now().toSec();
  double t = 0;
  float error[NUM_DOFS];
  float totalLinearError;
  float totalAngularError;
  float prevError[NUM_DOFS] = {0};
  float currentPoint;
  tf::StampedTransform ee_transform;
  double current_cart_pos[NUM_DOFS];
  double roll;
  double pitch;
  double yaw;
  kinova_msgs::PoseVelocity trajectoryPoint;
  ros::Rate rate(100);
  bool reachedFinalPoint;
  ros::Time finalPointTime;

  // Sending to the real robot
  while (!trajectoryComplete)
  {
    try{
      tf_listener.lookupTransform("/linear_actuator_link", side_ +"_ee_base", ros::Time(0), ee_transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    ee_transform.getBasis().getRPY(roll, pitch, yaw);
    current_cart_pos[0] = ee_transform.getOrigin().x();
    current_cart_pos[1] = ee_transform.getOrigin().y();
    current_cart_pos[2] = ee_transform.getOrigin().z();
    current_cart_pos[3] = simplify_angle(roll);
    current_cart_pos[4] = simplify_angle(pitch);
    current_cart_pos[5] = simplify_angle(yaw);

    //check for preempt requests from clients
    if (smoothTrajectoryServer.isPreemptRequested())
    {
      //stop gripper control
      trajectoryPoint.twist_linear_x = 0.0;
      trajectoryPoint.twist_linear_y = 0.0;
      trajectoryPoint.twist_linear_z = 0.0;
      trajectoryPoint.twist_angular_x = 0.0;
      trajectoryPoint.twist_angular_y = 0.0;
      trajectoryPoint.twist_angular_z = 0.0;

      cartesianCmdPublisher.publish(trajectoryPoint);

      //preempt action server
      smoothTrajectoryServer.setPreempted();
      ROS_INFO("Smooth trajectory server preempted by client");

      return;
    }

    //get time for trajectory
    t = ros::Time::now().toSec() - startTime;
    if (t > timePoints.at(timePoints.size() - 1))
    {
      //use final trajectory point as the goal to calculate error until the error is small enough to be considered successful

      if (!reachedFinalPoint)
      {
        reachedFinalPoint = true;
        finalPointTime = ros::Time::now();
      }

      totalLinearError = 0;
      totalAngularError = 0;
      for (unsigned int i = 0; i < NUM_DOFS; i++)
      {
        error[i] = splines.at(i)(timePoints.at(timePoints.size() - 1)) - current_cart_pos[i];

        if (i < 3) 
          totalLinearError += fabs(error[i]);
        else 
          totalAngularError += fabs(error[i]);
      }

      if ((totalLinearError < 0.01 && totalAngularError < .015) || ros::Time::now() - finalPointTime >= ros::Duration(3.0))

      {
        //stop arm
        trajectoryPoint.twist_linear_x = 0.0;
        trajectoryPoint.twist_linear_y = 0.0;
        trajectoryPoint.twist_linear_z = 0.0;
        trajectoryPoint.twist_angular_x = 0.0;
        trajectoryPoint.twist_angular_y = 0.0;
        trajectoryPoint.twist_angular_z = 0.0;
        cartesianCmdPublisher.publish(trajectoryPoint);
        trajectoryComplete = true;
        ROS_INFO("Trajectory complete!");
        break;
      }
    }
    else
    {
      for (unsigned int i = 0; i < NUM_DOFS; i++)
      {
        /* Linear Values*/
        if (i<=2)
          error[i] = splines.at(i)(t) - current_cart_pos[i];

        /* Angular Values*/
        else error[i] = nearest_equivalent_angle(simplify_angle((splines.at(i))(timePoints.at(t))), current_cart_pos[i]) - current_cart_pos[i];
      }
    }

    //calculate control input
    //populate the velocity command
    trajectoryPoint.twist_linear_x = KP_LINEAR * error[0] + KV_LINEAR * (error[0] - prevError[0]);
    trajectoryPoint.twist_linear_y = KP_LINEAR * error[1] + KV_LINEAR * (error[1] - prevError[1]);
    trajectoryPoint.twist_linear_z = KP_LINEAR * error[2] + KV_LINEAR * (error[2] - prevError[2]);
    trajectoryPoint.twist_angular_x = (KP_ANGULAR * error[3] + KV_ANGULAR * (error[3] - prevError[3]) * RAD_TO_DEG);
    trajectoryPoint.twist_angular_y = (KP_ANGULAR * error[4] + KV_ANGULAR * (error[4] - prevError[4]) * RAD_TO_DEG);
    trajectoryPoint.twist_angular_z = (KP_ANGULAR * error[5] + KV_ANGULAR * (error[5] - prevError[5]) * RAD_TO_DEG);

    //send the velocity command
    cartesianCmdPublisher.publish(trajectoryPoint);

    for (unsigned int i = 0; i < NUM_DOFS; i++)
      prevError[i] = error[i];

    rate.sleep();
    ros::spinOnce();
  }

  control_msgs::FollowJointTrajectoryResult result;
  result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
  smoothTrajectoryServer.setSucceeded(result);
}



// Spawn the node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_cartesian_trajectory_controller");

  JacoCartesianTrajectoryController jctc;
  ros::spin();
}

