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

        // 1. Dati Ottenuti dal Calcolo (EKF/TDoA) del drone osservatore
        Eigen::Vector3d estimated = observer->GetEstimatedPositionOf(sender_id);
        bool alarm = observer->IsAlarmActiveFor(sender_id);
        
        // 2. Verità Terreno (Ground Truth) del drone sender
        Eigen::Vector3d truth = sender->GetTruePosition();
        
        // 3. Calcolo Metriche
        // A. Discrepanza (Sicurezza): Quanto differisce il TDoA dal GPS dichiarato?
        double discrepancy = (estimated - claimed_gps).norm();

        // B. Errore di Stima (Performance): Quanto è preciso l'EKF rispetto alla realtà fisica?
        double estimation_error = (estimated - truth).norm();

        // 4. Scrittura su CSV

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
