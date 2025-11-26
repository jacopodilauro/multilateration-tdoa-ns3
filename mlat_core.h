#ifndef MLAT_CORE_H
#define MLAT_CORE_H

#include <vector>
#include <utility>
#include <Eigen/Dense>

Eigen::Vector3d select_best_solution(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2, 
                                     const Eigen::Vector3d& last_pos, bool has_last);

std::pair<bool, Eigen::Vector3d> mlat_solver(const std::vector<Eigen::Vector3d>& anchors, const std::vector<double>& times, 
                                             const Eigen::Vector3d& last_pos, bool has_last);

#endif
