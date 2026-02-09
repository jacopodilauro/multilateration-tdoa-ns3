#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"

#include "Drone.h"
#include "UWBChannel.h"
#include "Trajectories.h"
#include "SimulationLogger.h" 
#include "UWBMessage.h"

#include <vector>
#include <fstream>
#include <iostream>
#include <random>

using namespace ns3;
using namespace std;
using namespace Eigen;

const int NUM_DRONES = 6;        
const double SLOT_DURATION = 0.005;
const double SIM_TIME = 300.0;    
const double TIME_OF_MALICIOUS = 200.0;
// ID del drone che usiamo come orologio di riferimento (Master)
const int MASTER_ANCHOR_ID = 1;

class TDMAScheduler {
public:
    TDMAScheduler(vector<Ptr<Drone>>& swarm, Ptr<UWBChannel> channel, SimulationLogger& logger, ofstream& csv)
        : m_swarm(swarm), m_channel(channel), m_logger(logger), m_csv(csv), m_current_slot_idx(0) {}

    void Start() {
        ScheduleNextSlot();
    }

private:
    void ScheduleNextSlot() {
        Simulator::Schedule(Seconds(SLOT_DURATION), &TDMAScheduler::ExecuteSlot, this);
    }

    void ExecuteSlot() {
        double now = Simulator::Now().GetSeconds();
        if(now > SIM_TIME) return;

        int tx_idx = m_current_slot_idx % m_swarm.size();
        Ptr<Drone> sender = m_swarm[tx_idx];
        
        UWBMessage msg = sender->CreateTDMAMessage();
        Vector3d tx_pos_phys = sender->GetTruePosition(); 
        double tx_time_sec = msg.tx_timestamp_ps / 1e12; 

        // RICEZIONE FISICA & SINCRONIZZAZIONE
        vector<RangingMeasurement> shared_data_packet; 

        for(size_t i = 0; i < m_swarm.size(); ++i) {
            if((int)i == tx_idx) continue; 

            Ptr<Drone> rx = m_swarm[i];
            Vector3d rx_pos_phys = rx->GetTruePosition();
            
            ChannelCondition cond = m_channel->ComputeChannelCondition(tx_pos_phys, rx_pos_phys, 0.0);
            double dist = (tx_pos_phys - rx_pos_phys).norm();
            double c = 299792458.0;
            double tof = dist / c;
            
            // 1. SIMULAZIONE ERRORE FISICO (HARDWARE)
            // Aggiungiamo il drift fisico del ricevitore: questo creerebbe il disastro
            double rx_drift_physical = rx->GetClockDrift() * 1e-9; 
            double measured_toa_raw = now + tof + (cond.ranging_error_m / c) + rx_drift_physical;

            // 2. ALGORITMO DI SINCRONIZZAZIONE
            // Se il messaggio arriva dal Master (Drone 1), lo usiamo per correggere l'orologio
            if (tx_idx == MASTER_ANCHOR_ID) {
                // Calcoliamo quanto tempo ci avrebbe dovuto mettere basandoci sui GPS
                double geo_dist = (msg.gps_position - rx->GetGPSPosition()).norm();
                double expected_tof = geo_dist / c;
                
                // Quando doveva arrivare secondo l'orologio del Master (che è nel payload msg)
                double expected_arrival = tx_time_sec + expected_tof;
                
                // Differenza tra quando l'ho visto arrivare (raw) e quando doveva arrivare
                double clock_error = measured_toa_raw - expected_arrival;

                // Applichiamo un filtro semplice per non seguire troppo il rumore GPS
                // offset = offset_vecchio * 0.2 + errore_nuovo * 0.8
                double current_offset = rx->GetClockOffset();
                rx->SetClockOffset(current_offset * 0.2 + clock_error * 0.8);

                // Non usiamo il pacchetto del master per localizzare il master stesso in questo loop
                continue; 
            }

            // 3. CORREZIONE DEL DATO (SOFTWARE)
            // Per tutti gli altri messaggi (incluso il Target), applichiamo la correzione
            double corrected_toa = measured_toa_raw - rx->GetClockOffset();

            RangingMeasurement m;
            m.target_id = tx_idx;
            m.anchor_id = i;
            m.anchor_pos = rx->GetGPSPosition(); 
            m.toa_seconds = corrected_toa; // Usiamo il tempo sincronizzato
            m.is_los = cond.is_los;
            
            shared_data_packet.push_back(m);
        }

        // DATA LINK & COMPUTING 
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<> drop_chance(0.0, 1.0);
        double packet_loss_rate = 0.10;

        for(auto& drone : m_swarm) {
            if((int)drone->GetId() == tx_idx) continue;

            vector<RangingMeasurement> drone_specific_buffer;
            for(const auto& m : shared_data_packet) {
                if(m.anchor_id == drone->GetId()) {
                    drone_specific_buffer.push_back(m);
                } else {
                    if(drop_chance(gen) > packet_loss_rate) {
                        drone_specific_buffer.push_back(m);
                    }
                }
            }
            drone->ComputeNeighborPosition(tx_idx, msg.gps_position, drone_specific_buffer, now, tx_time_sec);
        }

// --- LOGICA CONSENSO E RECOVERY (SWARMRAFT) ---
        
        // 1. Fase di Votazione: calcoliamo la somma dei voti S_i 
        std::map<int, Vector3d> peer_estimates;
        int total_votes = 0;

// 1. Fase di Votazione: calcoliamo la somma dei voti S_i usando la Bitmask Radio
        for(auto& drone : m_swarm) {
            int observer_id = drone->GetId();
            if(observer_id == tx_idx) continue;
            peer_estimates[observer_id] = drone->GetEstimatedPositionOf(tx_idx);
            uint32_t mask = drone->GetVoteBitmask(); 
            bool vote_bit = (mask >> tx_idx) & 1; 
            total_votes += (vote_bit ? 1 : -1); 
        }
        Vector3d recovered_pos;
        if (total_votes <= -3) { 
            recovered_pos = m_swarm[0]->GetRecoveredPosition(tx_idx, peer_estimates);
			m_swarm[tx_idx]->ResetState(recovered_pos);
        } else {
            recovered_pos = msg.gps_position;
        }
        for(size_t i = 0; i < m_swarm.size(); ++i) {
            if((int)i == tx_idx) continue;
            m_logger.LogObservation(now, tx_idx, i, msg.gps_position, recovered_pos, m_csv);
        }

        m_current_slot_idx++;
        ScheduleNextSlot();
    }

    vector<Ptr<Drone>>& m_swarm;
    Ptr<UWBChannel> m_channel;
    SimulationLogger& m_logger;
    ofstream& m_csv;
    int m_current_slot_idx;
};

int main(int argc, char* argv[]) {
    
    Ptr<UWBChannel> channel = CreateObject<UWBChannel>();
    channel->SetEnvironment("outdoor"); 

    vector<Ptr<Drone>> swarm;
    for(int i = 0; i < NUM_DRONES; ++i) {
        Ptr<Drone> d = CreateObject<Drone>();
        d->SetId(i);
        swarm.push_back(d);
    }

    swarm[0]->SetTrajectory(GetTargetTrajectory());
    swarm[0]->SetMalicious(false);
    swarm[0]->SetClockDrift(10000.0); 
    cout << ">>> Configurazione: D0 Target (Spoofer) attivato." << endl;

    std::mt19937 init_rng(1234); 
    // Usiamo un drift ALTO (es. +/- 500ns) per provare che la sincronizzazione funziona!
    std::uniform_real_distribution<> drift_dist(-500.0, 500.0);

    for(int i = 1; i < NUM_DRONES; ++i) 
    {
        swarm[i]->SetTrajectory(GetAnchorTrajectory(i, NUM_DRONES)); 
        swarm[i]->SetMalicious(false);
        double d = drift_dist(init_rng);
        swarm[i]->SetClockDrift(d); // Hardware imperfetto
    }

    for(double t=0; t<=SIM_TIME; t+=0.05) 
    {
        Simulator::Schedule(Seconds(t), [swarm, t]()
        {
            for(auto& d : swarm) d->UpdatePosition(t);
        });
    }
    
    Simulator::Schedule(Seconds(TIME_OF_MALICIOUS), [swarm]()
	{
    // Attiva la modalità malevola sul drone 0 (Target)
    swarm[0]->SetMalicious(true); 
    std::cout << ">>> ATTACCO ATTIVATO: Il Drone 0 inizia lo spoofing GPS a t="<< TIME_OF_MALICIOUS <<"s <<<" << std::endl;
	});

    ofstream csv("tdma_security_log.csv");
    if(!csv.is_open()) return 1;
    // tdoa_uwb_sim.cpp (dentro main)
csv << "time,sender_id,observer_id,est_x,est_y,est_z,claim_x,claim_y,claim_z,true_x,true_y,true_z,discrepancy,estimation_error,alarm,rec_x,rec_y,rec_z\n";

    SimulationLogger logger(swarm);
    TDMAScheduler scheduler(swarm, channel, logger, csv);

    cout << "--- Avvio Simulazione RRS-TDoA-UWB (Synced) ---" << endl;
    scheduler.Start();

    Simulator::Stop(Seconds(SIM_TIME + 1.0));
    Simulator::Run();
    Simulator::Destroy();

    cout << "--- Fine. ---" << endl;

    return 0;
}
