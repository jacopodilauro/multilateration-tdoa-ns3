#include "UWBChannel.h"
#include <cmath>

using namespace ns3;
using namespace Eigen;
using namespace std;

TypeId UWBChannel::GetTypeId()
{
    static TypeId tid = TypeId("UWBChannel")
        .SetParent<Object>()
        .SetGroupName("UWB-Localization");
    return tid;
}

UWBChannel::UWBChannel()
    : m_environment("outdoor")
{
    std::random_device rd;
    m_rng.seed(rd());
}

UWBChannel::~UWBChannel() {}

void UWBChannel::SetEnvironment(std::string env_type) 
{
    m_environment = env_type;
}

void UWBChannel::AddObstacle(Vector3d center, double radius) 
{
    m_obstacles.push_back({center, radius});
}

bool UWBChannel::DetermineLOS(Vector3d tx, Vector3d rx) 
{
    Vector3d direction = (rx - tx).normalized();
    double total_distance = (rx - tx).norm();
    
    const int num_samples = 20;
    for (int i = 0; i < num_samples; ++i) {
        double t = (i / double(num_samples)) * total_distance;
        Vector3d point = tx + direction * t;
        
        for (const auto& obs : m_obstacles) {
            double dist_to_obstacle = (point - obs.first).norm();
            if (dist_to_obstacle < obs.second) {
                return false; 
            }
        }
    }
    
    double distance = total_distance;
    double p_los;
    
    if (m_environment == "outdoor") {
        p_los = std::exp(-distance / 500.0); 
    } else if (m_environment == "indoor") {
        p_los = std::exp(-distance / 30.0);
    } else {
        p_los = std::exp(-distance / 150.0);
    }
    
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    return uniform(m_rng) < p_los;
}

double UWBChannel::ComputePathLoss(double distance_m, bool is_los) 
{
    double freq_ghz = 6.5;
    double fspl_db = 20 * std::log10(distance_m) + 
                     20 * std::log10(freq_ghz) + 
                     92.45;
    
    if (is_los) {
        std::normal_distribution<double> shadow_fading(0.0, 3.0);
        return fspl_db + shadow_fading(m_rng);
    } else {
        double excess_pl = 0.0;
        if (m_environment == "outdoor") {
            excess_pl = 5.0 + 10 * std::log10(distance_m / 10.0);
        } else {
            excess_pl = 10.0 + 15 * std::log10(distance_m / 10.0);
        }
        
        std::normal_distribution<double> shadow_fading(0.0, 6.0); 
        return fspl_db + excess_pl + shadow_fading(m_rng);
    }
}

double UWBChannel::ComputeDelaySpread(double distance_m, bool is_los) 
{
    if (is_los) {
        return 5.0 + distance_m * 0.02;
    } else {
        if (m_environment == "outdoor") {
            return 15.0 + distance_m * 0.1;
        } else {
            return 25.0 + distance_m * 0.3;
        }
    }
}

double UWBChannel::ComputeRangingError(bool is_los, double distance_m) 
{
    double base_error;
    
    if (is_los) {
        std::normal_distribution<double> los_error(0.0, 0.10); 
        base_error = los_error(m_rng);
    } else {
        std::normal_distribution<double> nlos_variance(0.0, 0.50); 
        std::uniform_real_distribution<double> nlos_bias(0.3, 2.5); 
        
        base_error = nlos_bias(m_rng) + nlos_variance(m_rng);
    }
    
    double distance_factor = 1.0 + (distance_m / 200.0);
    return base_error * distance_factor;
}

ChannelCondition UWBChannel::ComputeChannelCondition(
    Vector3d tx_pos, 
    Vector3d rx_pos,
    double tx_power_dbm
) {
    ChannelCondition cond;
    double distance_m = (rx_pos - tx_pos).norm();
    
    cond.is_los = DetermineLOS(tx_pos, rx_pos);
    cond.path_loss_db = ComputePathLoss(distance_m, cond.is_los);
    cond.rssi_dbm = tx_power_dbm - cond.path_loss_db;
    cond.delay_spread_ns = ComputeDelaySpread(distance_m, cond.is_los);
    cond.ranging_error_m = ComputeRangingError(cond.is_los, distance_m);
    
    return cond;
}
