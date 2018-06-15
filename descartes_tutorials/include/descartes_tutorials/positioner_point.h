#ifndef POSITIONER_POINT_H
#define POSITIONER_POINT_H

#include "descartes_trajectory/cart_trajectory_pt.h"

class PositionerPoint : public descartes_core::TrajectoryPt
{
public:
  PositionerPoint(const Eigen::Affine3d& turn_table,
                  const Eigen::Affine3d& pt,
                  double turn_table_disc,
                  double turn_table_sym_tol,
                  double pt_disc,
                  double pt_sym_tol,
                  descartes_core::TimingConstraint timing);

  PositionerPoint(const Eigen::Affine3d& turn_table,
                  const Eigen::Affine3d& pt,
                  double turn_table_disc,
                  double pt_disc,
                  descartes_core::TimingConstraint timing);

  // TrajectoryPt interface
public:
  virtual bool getClosestCartPose(const std::vector<double> &seed_state, const descartes_core::RobotModel &kinematics, Eigen::Affine3d &pose) const override;
  virtual bool getNominalCartPose(const std::vector<double> &seed_state, const descartes_core::RobotModel &kinematics, Eigen::Affine3d &pose) const override;
  virtual void getCartesianPoses(const descartes_core::RobotModel &kinematics, EigenSTL::vector_Affine3d &poses) const override;
  virtual bool getClosestJointPose(const std::vector<double> &seed_state, const descartes_core::RobotModel &model, std::vector<double> &joint_pose) const override;
  virtual bool getNominalJointPose(const std::vector<double> &seed_state, const descartes_core::RobotModel &model, std::vector<double> &joint_pose) const override;
  virtual void getJointPoses(const descartes_core::RobotModel &model, std::vector<std::vector<double> > &joint_poses) const override;
  virtual bool isValid(const descartes_core::RobotModel &model) const override;
  virtual bool setDiscretization(const std::vector<double> &discretization) override;
  virtual descartes_core::TrajectoryPtPtr copy() const override;

private:
  Eigen::Affine3d turn_table_, pt_;
  double table_disc_, table_sym_tol_, pt_disc_, pt_sym_tol_;
};

#endif // POSITIONER_POINT_H
