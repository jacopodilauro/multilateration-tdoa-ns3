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

typedef std::function<Vector3d(double)> TrajectoryFunc;

class Drone : public Object {
public:
	void ResetState(Vector3d corrected_pos);
    Vector3d GetRecoveredPosition(int target_id, const std::map<int, 
                                  Vector3d>& all_peer_estimates);
    uint32_t GetVoteBitmask();

    static TypeId GetTypeId();
    Drone();
    virtual ~Drone();

    void SetId(uint32_t id);
    uint32_t GetId() const;

    void SetMalicious(bool is_malicious);
    bool IsMalicious();
    
    void SetInitialPosition(Vector3d pos);
    void SetTrajectory(TrajectoryFunc traj_func);
    void UpdatePosition(double time);
    
    Vector3d GetTruePosition() const;
    Vector3d GetGPSPosition();

    void SetClockDrift(double drift_ns);
    double GetClockDrift() const;
    void SetClockOffset(double offset);
    double GetClockOffset() const;
    UWBMessage CreateTDMAMessage();    
    void ComputeNeighborPosition(int sender_id, Vector3d claimed_gps, const vector<RangingMeasurement>& measurements, 
                                 double current_time, double tx_timestamp_sec);
    Vector3d GetEstimatedPositionOf(int target_id);
    bool IsAlarmActiveFor(int target_id);

private:
    uint32_t m_id;
    bool m_is_malicious;
    double m_clock_drift_ns;
    double m_clock_offset_correction;
    double m_attack_start_time;
    
    Vector3d m_true_position;
    TrajectoryFunc m_trajectory;

    std::mt19937 m_rng;
    std::normal_distribution<double> m_gps_noise_horiz;
    std::normal_distribution<double> m_gps_noise_vert;

    Vector3d AddGPSNoise(Vector3d true_pos);

    std::map<int, TDoAEKF> m_my_ekf_bank;
    std::map<int, double> m_last_calc_time;
    std::map<int, bool> m_security_alarm;
};

#endif
