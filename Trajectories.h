#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include <Eigen/Dense>
#include <functional>

using TrajectoryFunc = std::function<Eigen::Vector3d(double)>;

TrajectoryFunc GetAnchorTrajectory(int id, int total_anchors);
TrajectoryFunc GetTargetTrajectory();

#endif
