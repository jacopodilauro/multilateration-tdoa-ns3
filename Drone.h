#ifndef DRONE_H
#define DRONE_H

#include "ns3/core-module.h"
#include <Eigen/Dense>
#include <vector>
#include <functional>
#include <map>
#include "TDoAEKF.h"
#include "UWBMessage.h"
#include <random>
using namespace ns3;
using namespace Eigen;
using namespace std;

// Definiamo un tipo per la funzione di traiettoria
typedef std::function<Vector3d(double)> TrajectoryFunc;

class Drone : public Object {
public:
    static TypeId GetTypeId();
    Drone();
    virtual ~Drone();

    void SetId(uint32_t id);
    uint32_t GetId() const;

    void SetMalicious(bool is_malicious);
    
    // Gestione Movimento
    void SetInitialPosition(Vector3d pos);
    void SetTrajectory(TrajectoryFunc traj_func);
    void UpdatePosition(double time);
    
    Vector3d GetTruePosition() const;
    Vector3d GetGPSPosition(); 

    // Gestione Clock
    void SetClockDrift(double drift_ns);
    double GetClockDrift() const; // <--- NUOVO: Serve per simulare l'errore fisico

    // Gestione Sincronizzazione Software
    void SetClockOffset(double offset); // <--- NUOVO
    double GetClockOffset() const;      // <--- NUOVO

    // Comunicazione UWB
    UWBMessage CreateTDMAMessage();
    
    // Calcolo Distribuito
    void ComputeNeighborPosition(int sender_id, Vector3d claimed_gps, const vector<RangingMeasurement>& measurements, 
                                 double current_time, double tx_timestamp_sec);

    Vector3d GetEstimatedPositionOf(int target_id);
    bool IsAlarmActiveFor(int target_id);

private:
    uint32_t m_id;
    bool m_is_malicious;
    
    Vector3d m_true_position;
    TrajectoryFunc m_trajectory;
    
    // Parametri Clock
    double m_clock_drift_ns;        // Errore fisico hardware
    double m_clock_offset_correction; // Correzione software calcolata (Sync)

    // Rumore GPS
    std::mt19937 m_rng;
    std::normal_distribution<double> m_gps_noise_horiz;
    std::normal_distribution<double> m_gps_noise_vert;

    Vector3d AddGPSNoise(Vector3d true_pos);

    // Banca filtri EKF (uno per ogni vicino monitorato)
    std::map<int, TDoAEKF> m_my_ekf_bank;
    std::map<int, double> m_last_calc_time;
    std::map<int, bool> m_security_alarm;
};

#endif
