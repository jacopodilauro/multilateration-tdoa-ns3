#ifndef SIMULATION_LOGGER_H
#define SIMULATION_LOGGER_H

#include "ns3/core-module.h"
#include "Drone.h"
#include <vector>
#include <fstream>
#include <Eigen/Dense>

using namespace ns3;

class SimulationLogger {
public:
    SimulationLogger(std::vector<Ptr<Drone>>& swarm) : m_swarm(swarm) {}

    void LogObservation(
        double time, 
        int sender_id, 
        int observer_id,
        Eigen::Vector3d claimed_gps, 
               Eigen::Vector3d recovered_pos,
               std::ofstream& csv
    ) {
        
        Ptr<Drone> observer = m_swarm[observer_id];
        Ptr<Drone> sender = m_swarm[sender_id];

        Eigen::Vector3d estimated = observer->GetEstimatedPositionOf(sender_id);
        bool alarm = observer->IsAlarmActiveFor(sender_id);
        Eigen::Vector3d truth = sender->GetTruePosition();
        double discrepancy = (estimated - claimed_gps).norm();
        double estimation_error = (estimated - truth).norm();

        csv << time << "," << sender_id << "," << observer_id << ","
        << estimated.x() << "," << estimated.y() << "," << estimated.z() << ","
        << claimed_gps.x() << "," << claimed_gps.y() << "," << claimed_gps.z() << ","
        << truth.x() << "," << truth.y() << "," << truth.z() << ","
        << discrepancy << "," << estimation_error << "," << alarm << ","
        << recovered_pos.x() << "," << recovered_pos.y() << "," << recovered_pos.z() << "\n";
    }

private:
    std::vector<Ptr<Drone>>& m_swarm;
};

#endif
