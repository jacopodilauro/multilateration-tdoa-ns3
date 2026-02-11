/**
 * @author Jacopo Di Lauro, 2103045
 * @date 5th January 2026 (last important update)
 * 
 * University of Padua, Project of Cyber Security
 * 
 * Content of the Directory:
 * La cartella è divisa in diversi file:
 * tdoa_main: è il file principale in cui avviene la simulazione, il metodo TDMAScheduler gestisce tutta la logica temporale (Round Robin)
 *           Main
 *              1. Inizializzo il canale di comunicazione Ultra-WideBand (UWB) settando l'environment 'outdoor', il settaggio del tipo di 
 *                  ambiente modifica semplicemente il coefficiente di Packet Loss, se siamo al chiuso le distanze saranno minori e quindi anche 
 *                  un minor rischio di perdere informazioni. 
 *                  Il canale l'ho dovuto fare "custom" dato che non esistono moduli in ns3 uwb e online ho trovato solo una guida ma che non
 *                  sono riuscito a concretizzare. Per la realizzazione del canale mi sono fatto aiutare da l'LLM gemini. 
 *              2. Inizializzazione Sciame. Credo un insieme di oggetti Droni, settandoli tutti non malevoli, durante la simulazione però verrà settata
 *                 positiva la variabile booleana 'is_malicious' del drone 0 al tempo 200. Ogni drone è composto di un clock drift che sfaserà di pochi 
 *                 nanosecodi l'orologio del drone. Al dorne 0 viene settato un clockdrift fisso per controllare meglio il comportamento. Ogni drone 
 *                 dalla classe Trajecotry riceverà le coordinate di posizionamento a seconda della forma dello sciame, come spiegato nel paper
 *                 la forma migliore è la ottaedro. Dopo aver pianificato gli aggiornamenti della posizione di ogni drone ogni 0,05s inizio con la 
 *                 simulazione.
 *              3. Simulazione. Start() fa partire lo schedulerNexSlot  
 *
 *                
 *    
 * 
 */

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

        int tx_id = m_current_slot_idx % m_swarm.size();
        Ptr<Drone> sender = m_swarm[tx_id];
        
        UWBMessage msg = sender->CreateTDMAMessage();
        Vector3d tx_true_pos = sender->GetTruePosition(); 
        double tx_time_sec = msg.tx_timestamp_ps / 1e12; 
        vector<RangingMeasurement> shared_data_packet; 
        for(size_t i = 0; i < m_swarm.size(); ++i) {
            if((int)i == tx_id) continue; 

            Ptr<Drone> rx = m_swarm[i];
            Vector3d rx_pos_phys = rx->GetTruePosition();
            
            ChannelCondition cond = m_channel->ComputeChannelCondition(tx_true_pos, rx_pos_phys, 0.0);
            double dist = (tx_true_pos - rx_pos_phys).norm();
            double c = 299792458.0;
            double tof = dist / c;
            double rx_drift_physical = rx->GetClockDrift() * 1e-9; 
            double measured_toa_raw = now + tof + (cond.ranging_error_m / c) + rx_drift_physical;

            if (tx_id == MASTER_ANCHOR_ID) {
                double geo_dist = (msg.gps_position - rx->GetGPSPosition()).norm();
                double expected_tof = geo_dist / c;
                
                double expected_arrival = tx_time_sec + expected_tof;
                double clock_error = measured_toa_raw - expected_arrival;

                double current_offset = rx->GetClockOffset();
                rx->SetClockOffset(current_offset * 0.2 + clock_error * 0.8);
                continue; 
            }
            double corrected_toa = measured_toa_raw - rx->GetClockOffset();
            RangingMeasurement m;
            m.target_id = tx_id;
            m.anchor_id = i;
            m.anchor_pos = rx->GetGPSPosition(); 
            m.toa_seconds = corrected_toa; 
            m.is_los = cond.is_los;
            
            shared_data_packet.push_back(m);
        }

        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<> drop_chance(0.0, 1.0);
        double packet_loss_rate = 0.10;

        for(auto& drone : m_swarm) {
            if((int)drone->GetId() == tx_id) continue;

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
            drone->ComputeNeighborPosition(tx_id, msg.gps_position, drone_specific_buffer, now, tx_time_sec);
        }

// --- (SWARMRAFT) ---       
        std::map<int, Vector3d> peer_estimates;
        int total_votes = 0;
        for(auto& drone : m_swarm) {
            int observer_id = drone->GetId();
            if(observer_id == tx_id) continue;
            peer_estimates[observer_id] = drone->GetEstimatedPositionOf(tx_id);
            uint32_t mask = drone->GetVoteBitmask(); 
            bool vote_bit = (mask >> tx_id) & 1; 
            total_votes += (vote_bit ? 1 : -1); 
        }
        Vector3d recovered_pos;
        if (total_votes <= -3) { 
            recovered_pos = m_swarm[0]->GetRecoveredPosition(tx_id, peer_estimates);
			m_swarm[tx_id]->ResetState(recovered_pos);
        } else {
            recovered_pos = msg.gps_position;
        }
        for(size_t i = 0; i < m_swarm.size(); ++i) {
            if((int)i == tx_id) continue;
            m_logger.LogObservation(now, tx_id, i, msg.gps_position, recovered_pos, m_csv);
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
    cout << ">>> Finish Configuration." << endl;

    std::mt19937 init_rng(1234); 
    std::uniform_real_distribution<> drift_dist(-500.0, 500.0);

    for(int i = 1; i < NUM_DRONES; ++i) 
    {
        swarm[i]->SetTrajectory(GetAnchorTrajectory(i, NUM_DRONES)); 
        swarm[i]->SetMalicious(false);
        double d = drift_dist(init_rng);
        swarm[i]->SetClockDrift(d);
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
        swarm[0]->SetMalicious(true); 
        std::cout << ">>> ATTACK ACTIVATED: drone GPS spoofing <0> starts at t="<< TIME_OF_MALICIOUS <<"s <<<" << std::endl;
	});

    ofstream csv("tdma_security_log.csv");
    if(!csv.is_open()) return 1;
    csv << "time,sender_id,observer_id,est_x,est_y,est_z,claim_x,claim_y,claim_z,true_x,true_y,true_z,discrepancy,estimation_error,alarm,rec_x,rec_y,rec_z\n";

    SimulationLogger logger(swarm);
    TDMAScheduler scheduler(swarm, channel, logger, csv);

    cout << "--- Start Simulation RR-TDoA ---" << endl;
    
    scheduler.Start();
    Simulator::Stop(Seconds(SIM_TIME));
    Simulator::Run();
    Simulator::Destroy();
    cout << "--- End. ---" << endl;

    return 0;
}
