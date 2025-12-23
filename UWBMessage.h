#ifndef UWBMESSAGE_H
#define UWBMESSAGE_H

#include <Eigen/Dense> 

struct UWBMessage {
    uint32_t sender_id;         // Chi invia
    uint64_t tx_timestamp_ps;   // Timestamp invio
    Eigen::Vector3d gps_position;
};

// La misura presa da chi ascolta
struct RangingMeasurement {
    uint32_t target_id;         // Chi stiamo tracciando (il sender)
    uint32_t anchor_id;         // Chi ha ricevuto (il receiver)
    Eigen::Vector3d anchor_pos; // Dove si trova il receiver (GPS)
    double toa_seconds;         // Time of Arrival misurato
    bool is_los;                
};

#endif
