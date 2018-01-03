// Core ros functionality like ros::init and spin
#include <ros/ros.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

// Includes the descartes robot model we will be using
#include <descartes_opw_model/descartes_opw_model.h>

// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>

#include <opw_kinematics/opw_parameters_examples.h>
// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>

// Includes the utility function for converting to trajectory_msgs::JointTrajectory's
#include <descartes_utilities/ros_conversions.h>

#include "descartes_tutorials/positioner_point.h"

// For reading the position of the grinder from TF
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// For loading the pose file from a local package
#include <ros/package.h>
#include <fstream>
/**
 * Makes a dummy trajectory for the robot to follow.
 */
std::vector<descartes_core::TrajectoryPtPtr> makePath();

/**
 * Sends a ROS trajectory to the robot controller
 */
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "descartes_tutorial");
  ros::NodeHandle nh;

  // I won't be repeating too many comments, so please take a look back at the first tutorial for an explanation of
  // the basics.
  ros::AsyncSpinner spinner (1);
  spinner.start();

  // This package assumes that the move group you are using is pointing to an IKFast kinematics plugin in its
  // kinematics.yaml file. By default, it assumes that the underlying kinematics are from 'base_link' to 'tool0'.
  // We're using a group that does not begin or end at these links, but they do exist in the model.
  // If you have renamed these, please set the 'ikfast_base_frame' and 'ikfast_tool_frame' parameter (not in the
  // private namespace) to the base and tool frame used to generate the IKFast model.
  auto abb = opw_kinematics::makeIrb2400_10<double>();
  descartes_core::RobotModelPtr model (new descartes_opw_model::OPWMoveitStateAdapter(abb, "base_link", "tool0"));

  const std::string robot_description = "robot_description";

  // We have made a special move group which goes from the base link of the robot to the reference frame of the mounted
  // part that the robot is holding.
  const std::string group_name = "manipulator_pos";

  // Name of frame in which you are expressing poses.
  const std::string world_frame = "base_link";

  // In this demo, "part" is a reference frame located on the part that the robot is holding. It's a frame that we can
  // precisely locate w.r.t the robot, and in which we can export the poses representing the edge of the part.
  const std::string tcp_frame = "tool1";

  if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    ROS_INFO("Could not initialize robot model");
    return -1;
  }

  model->setCheckCollisions(true); // Let's turn on collision checking.

  // Here we load a path by reading a file of poses generated on the perimeter of the part being held by the robot.
  // These were generated from the CAD model. This is the interesting bit of this tutorial.
  std::vector<descartes_core::TrajectoryPtPtr> points = makePath();

  descartes_planner::DensePlanner planner;

  if (!planner.initialize(model))
  {
    ROS_ERROR("Failed to initialize planner");
    return -2;
  }

  auto start_tm = ros::WallTime::now();
  if (!planner.planPath(points))
  {
    ROS_ERROR("Could not solve for a valid path");
    return -3;
  }

  auto search_start_tm = ros::WallTime::now();
  std::vector<descartes_core::TrajectoryPtPtr> result;
  if (!planner.getPath(result))
  {
    ROS_ERROR("Could not retrieve path");
    return -4;
  }

  auto end_tm = ros::WallTime::now();

  ROS_INFO_STREAM("Graph building (s): " << (search_start_tm - start_tm).toSec());
  ROS_INFO_STREAM("Graph search (s): " << (end_tm - search_start_tm).toSec());
  ROS_INFO_STREAM("Total time (s): " << (end_tm - start_tm).toSec());

  // Translate the result into something that you can execute.
  std::vector<std::string> names;
  nh.getParam("controller_joint_names", names);

  // Create a JointTrajectory
  trajectory_msgs::JointTrajectory joint_solution;
  joint_solution.joint_names = names;

  // Define a default velocity. Descartes points without specified timing will use this value to limit the
  // fastest moving joint. This usually effects the first point in your path the most.
  const static double default_joint_vel = 0.5; // rad/s
  if (!descartes_utilities::toRosJointPoints(*model, result, default_joint_vel, joint_solution.points))
  {
    ROS_ERROR("Unable to convert Descartes trajectory to joint points");
    return -5;
  }

  // 6. Send the ROS trajectory to the robot for execution
  if (!executeTrajectory(joint_solution))
  {
    ROS_ERROR("Could not execute trajectory!");
    return -6;
  }

  // Wait till user kills the process (Control-C)
  ROS_INFO("Done!");
  return 0;
}

std::vector<descartes_core::TrajectoryPtPtr> makePoses()
{
  std::vector<descartes_core::TrajectoryPtPtr> out;
  tf::TransformListener listener;

  tf::StampedTransform p;
  listener.waitForTransform("base_link", "positioner_base", ros::Time(), ros::Duration(5.0));
  listener.lookupTransform("base_link", "positioner_base", ros::Time(), p);

  Eigen::Affine3d base_pose;
  tf::poseTFToEigen(p, base_pose);

  Eigen::Vector3d box_dims (0.25, 0.5, 0.5);

  for (double y = -0.25; y < 0.25; y += 0.02)
  {
    Eigen::Affine3d i;
    i.setIdentity();
    i.translation() = Eigen::Vector3d(-0.25/2.0, y, 0.5);

    i *= Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());

    descartes_core::TrajectoryPtPtr pt (new PositionerPoint(base_pose, i, 0.01, 0.5, descartes_core::TimingConstraint(0.1)));
    out.push_back(pt);
  }

  for (double x = -0.25/2.; x < 0.25/2.; x += 0.02)
  {
    Eigen::Affine3d i;
    i.setIdentity();
    i.translation() = Eigen::Vector3d(x, 0.25, 0.5);

    i *= Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());

    descartes_core::TrajectoryPtPtr pt (new PositionerPoint(base_pose, i, 0.01, 0.5, descartes_core::TimingConstraint(0.1)));
    out.push_back(pt);
  }

  for (double y =0.25; y > -0.25; y -= 0.02)
  {
    Eigen::Affine3d i;
    i.setIdentity();
    i.translation() = Eigen::Vector3d(0.25/2, y, 0.5);

    i *= Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());

    descartes_core::TrajectoryPtPtr pt (new PositionerPoint(base_pose, i, 0.01, 0.5, descartes_core::TimingConstraint(0.1)));
    out.push_back(pt);
  }

  for (double x = 0.25/2.; x > -0.25/2.; x -= 0.02)
  {
    Eigen::Affine3d i;
    i.setIdentity();
    i.translation() = Eigen::Vector3d(x, -0.25, 0.5);

    i *= Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());

    descartes_core::TrajectoryPtPtr pt (new PositionerPoint(base_pose, i, 0.01, 0.5, descartes_core::TimingConstraint(0.1)));
    out.push_back(pt);
  }

  return out;
}

descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose, double dt)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose), TimingConstraint(dt)) );
}

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose, double dt)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI / 12.0, AxialSymmetricPt::Z_AXIS, TimingConstraint(dt)) );
}



std::vector<descartes_core::TrajectoryPtPtr> makePath()
{
  return makePoses();
}

bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac ("joint_trajectory_action", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(1.0);

  return ac.sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED;
}
