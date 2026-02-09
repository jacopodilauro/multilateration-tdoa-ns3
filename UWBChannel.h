#ifndef UWB_CHANNEL_H
#define UWB_CHANNEL_H

#include "ns3/core-module.h"
#include "UWBMessage.h"
#include <Eigen/Dense>
#include <random>
#include <vector>
#include <cstdint>

using namespace ns3;
using namespace Eigen;

struct ChannelCondition {
    bool is_los;              
    double path_loss_db;      
    double delay_spread_ns;   
    double rssi_dbm;          
    double ranging_error_m;   
};

class UWBChannel : public Object
{
public:
    static TypeId GetTypeId();
    
    UWBChannel();
    virtual ~UWBChannel();
    
    ChannelCondition ComputeChannelCondition(
        Vector3d tx_pos, 
        Vector3d rx_pos,
        double tx_power_dbm = 0.0
    );
    
    void SetEnvironment(std::string env_type); 
    void AddObstacle(Vector3d center, double radius); 
    
private:
    std::string m_environment;
    std::vector<std::pair<Vector3d, double>> m_obstacles; 
    std::mt19937 m_rng;
    
    bool DetermineLOS(Vector3d tx, Vector3d rx);
    double ComputePathLoss(double distance_m, bool is_los);
    double ComputeDelaySpread(double distance_m, bool is_los);
    double ComputeRangingError(bool is_los, double distance_m);
};

#endif
