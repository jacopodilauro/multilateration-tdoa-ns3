#include "Drone.h"
#include <iostream>
#include <cmath>

using namespace ns3;
using namespace Eigen;
using namespace std;

TypeId Drone::GetTypeId() {
    static TypeId tid = TypeId("Drone")
        .SetParent<Object>()
        .SetGroupName("UWB-Localization");
    return tid;
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
    //if (m_is_malicious) noisy.y() += 15.0; 
    //modify
    if (m_is_malicious) {
        // --- CONFIGURAZIONE RAMPA ---
        const double TARGET_OFFSET = 15.0; // Offset finale in metri (Y)
        const double RAMP_DURATION = 10.0; // Durata della transizione in secondi
        // ----------------------------

        double now = Simulator::Now().GetSeconds();
        
        // Calcola da quanto tempo è attivo l'attacco
        double time_elapsed = now - m_attack_start_time;

        // Calcola la percentuale di completamento (da 0.0 a 1.0)
        double progress = time_elapsed / RAMP_DURATION;
        
        // Clamping: assicuriamoci che stia tra 0 e 1
        if (progress < 0.0) progress = 0.0;
        if (progress > 1.0) progress = 1.0;

        // Applica l'offset scalato
        noisy.y() += (TARGET_OFFSET * progress); 
    }
    //end
    return noisy;
}

Vector3d Drone::GetTruePosition() const { return m_true_position; }

// --- GESTIONE CLOCK ---
void Drone::SetClockDrift(double drift_ns) { m_clock_drift_ns = drift_ns; }
double Drone::GetClockDrift() const { return m_clock_drift_ns; }

void Drone::SetClockOffset(double offset) { m_clock_offset_correction = offset; }
double Drone::GetClockOffset() const { return m_clock_offset_correction; }
// ----------------------

UWBMessage Drone::CreateTDMAMessage() {
    UWBMessage msg;
    msg.sender_id = m_id;
    msg.gps_position = GetGPSPosition();
    
    // Drift in trasmissione (fisico)
    uint64_t drift_ps = (uint64_t)(m_clock_drift_ns * 1000.0);
    msg.tx_timestamp_ps = Simulator::Now().GetPicoSeconds() + drift_ps; 
    
    return msg;
}

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
    
    // Servono almeno 4 ancore per risolvere x,y,z + bias
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
