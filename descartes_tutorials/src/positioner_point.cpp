#include "descartes_tutorials/positioner_point.h"

PositionerPoint::PositionerPoint(const Eigen::Affine3d &turn_table,
                                 const Eigen::Affine3d &pt,
                                 double turn_table_disc,
                                 double turn_table_sym_tol,
                                 double pt_disc,
                                 double pt_sym_tol,
                                 descartes_core::TimingConstraint timing)
  : TrajectoryPt(timing),
    turn_table_(turn_table),
    pt_(pt),
    table_disc_(turn_table_disc),
    table_sym_tol_(turn_table_sym_tol),
    pt_disc_(pt_disc),
    pt_sym_tol_(pt_sym_tol)
{
}

PositionerPoint::PositionerPoint(const Eigen::Affine3d &turn_table,
                                 const Eigen::Affine3d &pt,
                                 double turn_table_disc,
                                 double pt_disc,
                                 descartes_core::TimingConstraint timing)
  : TrajectoryPt(timing),
    turn_table_(turn_table),
    pt_(pt),
    table_disc_(turn_table_disc),
    table_sym_tol_(M_PI),
    pt_disc_(pt_disc),
    pt_sym_tol_(M_PI)
{
}

bool PositionerPoint::getClosestCartPose(const std::vector<double> &seed_state, const descartes_core::RobotModel &kinematics, Eigen::Affine3d &pose) const
{

}

bool PositionerPoint::getNominalCartPose(const std::vector<double> &seed_state, const descartes_core::RobotModel &kinematics, Eigen::Affine3d &pose) const
{

}

void PositionerPoint::getCartesianPoses(const descartes_core::RobotModel &kinematics, EigenSTL::vector_Affine3d &poses) const
{

}

bool PositionerPoint::getClosestJointPose(const std::vector<double> &seed_state, const descartes_core::RobotModel &model, std::vector<double> &joint_pose) const
{

}

bool PositionerPoint::getNominalJointPose(const std::vector<double> &seed_state, const descartes_core::RobotModel &model, std::vector<double> &joint_pose) const
{

}

void PositionerPoint::getJointPoses(const descartes_core::RobotModel &model, std::vector<std::vector<double> > &joint_poses) const
{
  std::vector<std::vector<double>> results;
  for (double table = -table_sym_tol_; table < table_sym_tol_; table += table_disc_)
  {
    for (double angle = -pt_sym_tol_; angle < pt_sym_tol_; angle += pt_disc_)
    {
      Eigen::Affine3d pose = turn_table_ * Eigen::AngleAxisd(table, Eigen::Vector3d::UnitZ()) * pt_ * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
      model.getAllIK(pose, results);
      for (auto& s : results)
      {
        s[6] = table;
        if (model.isValid(s))
        {
          joint_poses.push_back(s);
        }
      }

    }
  }
}

bool PositionerPoint::isValid(const descartes_core::RobotModel &model) const
{

}

bool PositionerPoint::setDiscretization(const std::vector<double> &discretization)
{

}

descartes_core::TrajectoryPtPtr PositionerPoint::copy() const
{

}
