#include "Drone.h"
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace ns3;
using namespace Eigen;
using namespace std;

TypeId Drone::GetTypeId() {
    static TypeId tid = TypeId("Drone")
        .SetParent<Object>()
        .SetGroupName("UWB-Localization");
    return tid;
}

void Drone::ResetState(Vector3d recovered_pos) {
	const double alpha = 0.15; 
	m_true_position = (1.0 - alpha) * m_true_position + alpha * recovered_pos;
    double dist = (m_true_position - recovered_pos).norm();
    if (dist < 1.0) {
            for (auto& pair : m_security_alarm) pair.second = false;
        }
}

uint32_t Drone::GetVoteBitmask() {
    uint32_t mask = 0xFFFFFFFF; 
    for (auto const& [id, is_alarm] : m_security_alarm) {
        if (is_alarm) mask &= ~(1 << id);
    }
    return mask;
}

UWBMessage Drone::CreateTDMAMessage() {
    UWBMessage msg;
    msg.sender_id = m_id;
    msg.gps_position = GetGPSPosition();
    uint32_t mask = 0xFFFFFFFF; 
    for (auto const& [target_id, is_alarmed] : m_security_alarm) {
        if (is_alarmed) {
            mask &= ~(1 << target_id); 
        }
    }
    msg.vote_bitmask = mask;

    uint64_t drift_ps = (uint64_t)(m_clock_drift_ns * 1000.0);
    msg.tx_timestamp_ps = Simulator::Now().GetPicoSeconds() + drift_ps; 
    
    return msg;
}

Drone::Drone() : m_id(0), m_is_malicious(false), m_clock_drift_ns(0.0), m_clock_offset_correction(0.0), m_attack_start_time(0.0) {
    std::random_device rd;
    m_rng.seed(rd());
    m_gps_noise_horiz = std::normal_distribution<double>(0.0, 0.05); 
    m_gps_noise_vert = std::normal_distribution<double>(0.0, 0.10); 
}

Drone::~Drone() {}

void Drone::SetId(uint32_t id) { m_id = id; }
uint32_t Drone::GetId() const { return m_id; }

void Drone::SetMalicious(bool is_malicious) 
{ 
	if (is_malicious && !m_is_malicious) 
	{
        m_attack_start_time = Simulator::Now().GetSeconds();
    }
	m_is_malicious = is_malicious; 
}
void Drone::SetInitialPosition(Vector3d pos) { m_true_position = pos; }
void Drone::SetTrajectory(std::function<Vector3d(double)> traj_func) { m_trajectory = traj_func; }
bool Drone::IsMalicious() { return m_is_malicious; }

Vector3d Drone::GetRecoveredPosition(int target_id, const std::map<int, Vector3d>& all_peer_estimates) {
    std::vector<double> x_coords, y_coords, z_coords;

    for (auto const& [peer_id, estimate] : all_peer_estimates) {
        x_coords.push_back(estimate.x());
        y_coords.push_back(estimate.y());
        z_coords.push_back(estimate.z());
    }

    if (x_coords.empty()) return Vector3d(0,0,0);

    auto findMedian = [](std::vector<double>& v) {
        size_t n = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + n, v.end());
        return v[n];
    };
     
    return Vector3d(
        findMedian(x_coords),
        findMedian(y_coords),
        findMedian(z_coords)
    );
}

void Drone::UpdatePosition(double time) {
    if (m_trajectory) m_true_position = m_trajectory(time);
}

Vector3d Drone::AddGPSNoise(Vector3d true_pos) {
    return Vector3d(
        true_pos.x() + m_gps_noise_horiz(m_rng),
        true_pos.y() + m_gps_noise_horiz(m_rng),
        true_pos.z() + m_gps_noise_vert(m_rng)
    );
}

Vector3d Drone::GetGPSPosition() {
    Vector3d noisy = AddGPSNoise(m_true_position);
    if (m_is_malicious) {
        const double TARGET_OFFSET = 15.0; 
        const double RAMP_DURATION = 10.0; 

        double now = Simulator::Now().GetSeconds();
        double time_elapsed = now - m_attack_start_time;
        double progress = time_elapsed / RAMP_DURATION;
        if (progress < 0.0) progress = 0.0;
        if (progress > 1.0) progress = 1.0;
        noisy.y() += (TARGET_OFFSET * progress); 
    }
    return noisy;
}

Vector3d Drone::GetTruePosition() const { return m_true_position; }

void Drone::SetClockDrift(double drift_ns) { m_clock_drift_ns = drift_ns; }
double Drone::GetClockDrift() const { return m_clock_drift_ns; }

void Drone::SetClockOffset(double offset) { m_clock_offset_correction = offset; }
double Drone::GetClockOffset() const { return m_clock_offset_correction; }

void Drone::ComputeNeighborPosition(int sender_id, Vector3d claimed_gps, const vector<RangingMeasurement>& measurements,
    								double current_time, double tx_timestamp_sec) 
{
    if ((int)m_id == sender_id) return;

    if (m_my_ekf_bank.find(sender_id) == m_my_ekf_bank.end()) 
    {
        m_my_ekf_bank[sender_id].Init(claimed_gps); 
        m_last_calc_time[sender_id] = 0.0;
        m_security_alarm[sender_id] = false;
    }
    
    if(measurements.size() < 4) return;

    vector<TDoAEKF::Msmnt> input_data;
    for(const auto& m : measurements) {
        TDoAEKF::Msmnt data;
        data.anchor_pos = m.anchor_pos; 
        data.toa = m.toa_seconds;       
        data.tx_timestamp = tx_timestamp_sec; 
        
        input_data.push_back(data);
    }

    double dt = current_time - m_last_calc_time[sender_id];
    if (dt > 0) 
    {
        m_my_ekf_bank[sender_id].Predict(dt);
        m_my_ekf_bank[sender_id].Update(input_data);
        m_last_calc_time[sender_id] = current_time;
    }

    Vector3d calculated_pos = m_my_ekf_bank[sender_id].GetPosition();
    double error = (calculated_pos - claimed_gps).norm();
    
    if (error > 10.0) m_security_alarm[sender_id] = true;
    else m_security_alarm[sender_id] = false;
}

Vector3d Drone::GetEstimatedPositionOf(int target_id) {
    if (m_my_ekf_bank.count(target_id)) return m_my_ekf_bank[target_id].GetPosition();
    return Vector3d(0,0,0);
}

bool Drone::IsAlarmActiveFor(int target_id) {
    if (m_security_alarm.count(target_id)) return m_security_alarm[target_id];
    return false;
}