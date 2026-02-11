#ifndef UWBMESSAGE_H
#define UWBMESSAGE_H

#include <Eigen/Dense> 

struct UWBMessage {
    uint32_t sender_id;         
    uint64_t tx_timestamp_ps;
    Eigen::Vector3d gps_position;
    uint32_t vote_bitmask;
};

struct RangingMeasurement {
    uint32_t target_id;
    uint32_t anchor_id;
    Eigen::Vector3d anchor_pos;
    double toa_seconds;
    bool is_los;                
};

#endif
