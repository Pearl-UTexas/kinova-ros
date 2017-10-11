#include <hlpr_cartesian_trajectory_controller_two_arms.h>

using namespace std;

// Initialization
JacoCartesianTrajectoryController::JacoCartesianTrajectoryController() : pnh("~"), smoothTrajectoryServer(pnh, "cart_trajectory", boost::bind(&JacoCartesianTrajectoryController::executeSmoothTrajectory, this, _1), false)
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
  use_time_service_ = n.advertiseService("use_custom_time", &JacoTrajectoryController::useTimeService, this);

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
static inline double nearest_equivalent(double desired, double current)
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
    {
      timePoints[i] = goal->trajectory.points.at(i).time_from_start.toSec();
    }
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
  float totalError;
  float prevError[NUM_DOFS] = {0};
  float currentPoint;
  double current_cart_pos[NUM_DOFS];
  kinova_msgs::PoseVelocity trajectoryPoint;
  ros::Rate rate(100);
  bool reachedFinalPoint;
  ros::Time finalPointTime;

  // Sending to the real robot
  while (!trajectoryComplete)
  {
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

      // for (unsigned int i = 0; i < NUM_DOFS; i++)
      // {
      //   current_joint_pos[i] = jointStates.position[i];
      // }
      tf::StampedTransform transform;
      listener.lookupTransform("/linear_actuator_link", side_ +"_ee_link", ros::Time(0), transform);

      totalError = 0;
      for (unsigned int i = 0; i < NUM_DOFS; i++)
      {
        currentPoint = simplify_angle(current_joint_pos[i]);
        error[i] = nearest_equivalent(simplify_angle((splines.at(i))(timePoints.at(timePoints.size() - 1))),
                                      currentPoint) - currentPoint;
        totalError += fabs(error[i]);
      }


      if (totalError < .035 || ros::Time::now() - finalPointTime >= ros::Duration(3.0))

      {
        //stop arm
        trajectoryPoint.joint1 = 0.0;
        trajectoryPoint.joint2 = 0.0;
        trajectoryPoint.joint3 = 0.0;
        trajectoryPoint.joint4 = 0.0;
        trajectoryPoint.joint5 = 0.0;
        trajectoryPoint.joint6 = 0.0;
        trajectoryPoint.joint7 = 0.0;
        angularCmdPublisher.publish(trajectoryPoint);
        trajectoryComplete = true;
        ROS_INFO("Trajectory complete!");
        break;
      }
    }
    else
    {
      //calculate error
      /*
      {
        boost::recursive_mutex::scoped_lock lock(api_mutex);
        GetAngularPosition(position_data);
      }
      */
      for (unsigned int i = 0; i < NUM_DOFS; i++)
      {
        current_joint_pos[i] = jointStates.position[i];
      }

      for (unsigned int i = 0; i < NUM_DOFS; i++)
      {
        currentPoint = simplify_angle(current_joint_pos[i]);
        error[i] = nearest_equivalent(simplify_angle((splines.at(i))(t)), currentPoint) - currentPoint;
      }
    }

    //calculate control input
    //populate the velocity command
    trajectoryPoint.joint1 = (KP * error[0] + KV * (error[0] - prevError[0]) * RAD_TO_DEG);
    trajectoryPoint.joint2 = (KP * error[1] + KV * (error[1] - prevError[1]) * RAD_TO_DEG);
    trajectoryPoint.joint3 = (KP * error[2] + KV * (error[2] - prevError[2]) * RAD_TO_DEG);
    trajectoryPoint.joint4 = (KP * error[3] + KV * (error[3] - prevError[3]) * RAD_TO_DEG);
    trajectoryPoint.joint5 = (KP * error[4] + KV * (error[4] - prevError[4]) * RAD_TO_DEG);
    trajectoryPoint.joint6 = (KP * error[5] + KV * (error[5] - prevError[5]) * RAD_TO_DEG);
    trajectoryPoint.joint7 = (KP * error[6] + KV * (error[6] - prevError[6]) * RAD_TO_DEG);

    //for debugging:
    // cout << "Errors: " << error[0] << ", " << error[1] << ", " << error[2] << ", " << error[3] << ", " << error[4] << ", " << error[5] << endl;

    //send the velocity command
    angularCmdPublisher.publish(trajectoryPoint);

    for (unsigned int i = 0; i < NUM_DOFS; i++)
    {
      prevError[i] = error[i];
    }

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

