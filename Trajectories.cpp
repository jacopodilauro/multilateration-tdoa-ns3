#include "Trajectories.h"
#include <cmath>
#include <iostream>

using namespace Eigen;
using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =========================================================
// CONFIGURAZIONE SCENARIO
// =========================================================
// 1 = ATOMIC SHELL
// 2 = CIRCULAR PATROL
// 3 = OCTAHEDRON SWARM (Tutti traslano insieme)
const int SCENARIO_TYPE = 3; 

// Parametri Geometria
const double RADIUS = 80.0;              
const double SPEED_FACTOR = 1.0;         

// =========================================================
// PARAMETRI DI TRASLAZIONE GLOBALE (MOVIMENTO DI GRUPPO)
// =========================================================
const Vector3d INITIAL_CENTER(100.0, 0.0, 50.0);

// Velocità di traslazione dello sciame in m/s (x, y, z)
// Modifica questi valori per cambiare la direzione del gruppo
const Vector3d SWARM_VELOCITY(2.5, 0.0, 0.0); 

// Funzione helper per calcolare dove si trova il centro dello sciame al tempo t
Vector3d GetCurrentCenter(double t) {
    // Posizione = PosizioneIniziale + (Velocità * Tempo)
    return INITIAL_CENTER + (SWARM_VELOCITY * t);
}

// =========================================================
// DEFINIZIONE TRAIETTORIE
// =========================================================

// --- SCENARIO 1: ATOMIC SHELL ---
TrajectoryFunc MakeAtomicShell(int id, int total) {
    double inclination = 0.0;
    if (id % 3 == 0) inclination = 0.0;          
    else if (id % 3 == 1) inclination = M_PI/2;  
    else inclination = M_PI/4;                   

    double phase_offset = (id * 2.0 * M_PI) / total;

    return [inclination, phase_offset](double t) -> Vector3d {
        Vector3d center = GetCurrentCenter(t); // Centro dinamico

        double omega = ((2.0 * M_PI) / 50.0) * SPEED_FACTOR;
        double angle = omega * t + phase_offset;

        double base_x = RADIUS * cos(angle);
        double base_y = RADIUS * sin(angle);
        double base_z = 0.0;

        // Rotazione 3D
        double x = base_x;
        double y = base_y * cos(inclination) - base_z * sin(inclination);
        double z = base_y * sin(inclination) + base_z * cos(inclination);

        if (inclination == 0.0) z += 15.0 * sin(angle * 2.0); 

        return Vector3d(center.x() + x, center.y() + y, center.z() + z);
    };
}

// --- SCENARIO 2: CIRCULAR PATROL ---
TrajectoryFunc MakeCircularPatrol(int id, int total) {
    double angle_step = (2.0 * M_PI) / (double)(total - 1);
    double start_angle = (id - 1) * angle_step;

    return [start_angle, id](double t) -> Vector3d {
        Vector3d center = GetCurrentCenter(t); // Centro dinamico

        double omega = ((2.0 * M_PI) / 60.0) * SPEED_FACTOR;
        double current_angle = start_angle + (omega * t);

        double x = center.x() + RADIUS * cos(current_angle);
        double y = center.y() + RADIUS * sin(current_angle);
        double z = center.z() + ((id % 2 == 0) ? 15.0 : -15.0); 

        return Vector3d(x, y, z);
    };
}

// --- SCENARIO 3: OCTAHEDRON (Tutti traslano insieme) ---
TrajectoryFunc MakeOctahedronFormation(int id) {
    return [id](double t) -> Vector3d {
        // 1. Otteniamo la posizione attuale del centro dello sciame
        Vector3d center = GetCurrentCenter(t);

        double omega = 0.2 * SPEED_FACTOR; 
        double theta = omega * t;
        
        double local_x = 0, local_y = 0, local_z = 0;

        // Mappatura ID -> Vertici Ottaedro (Coordinate Locali)
        switch(id % 6) { 
            case 0: local_z = RADIUS; break;  // Zenit (Ora si muove con center)
            case 1: local_z = -RADIUS; break; // Nadir (Ora si muove con center)
            case 2: 
                local_x = RADIUS * cos(theta); 
                local_y = RADIUS * sin(theta); 
                break;
            case 3: 
                local_x = RADIUS * cos(theta + M_PI/2); 
                local_y = RADIUS * sin(theta + M_PI/2); 
                break;
            case 4: 
                local_x = RADIUS * cos(theta + M_PI); 
                local_y = RADIUS * sin(theta + M_PI); 
                break;
            case 5: 
                local_x = RADIUS * cos(theta + 3*M_PI/2); 
                local_y = RADIUS * sin(theta + 3*M_PI/2); 
                break;
        }

        // Oscillazione locale (solo equatore)
        if (id >= 2) local_z += 10.0 * sin(t * 0.5 + id);

        // 2. Sommiamo coordinate locali al centro globale dinamico
        return Vector3d(center.x() + local_x, center.y() + local_y, center.z() + local_z);
    };
}

// --- TRAIETTORIA TARGET (Figura a 8) ---
TrajectoryFunc MakeFigureEightTarget() {
    return [](double t) -> Vector3d {
        Vector3d center = GetCurrentCenter(t); // Anche il target segue il flusso del gruppo

        double omega = ((2.0 * M_PI) / 40.0) * SPEED_FACTOR;
        double x = 70.0 * sin(omega * t);
        double y = 30.0 * sin(2.0 * omega * t);
        double z = 10.0 * cos(omega * t);
        
        return Vector3d(center.x() + x, center.y() + y, center.z() + z);
    };
}

// =========================================================
// DISPATCHER
// =========================================================

TrajectoryFunc GetAnchorTrajectory(int id, int total) {
    if (SCENARIO_TYPE == 3) {
        return MakeOctahedronFormation(id); 
    } else if (SCENARIO_TYPE == 2) {
        return MakeCircularPatrol(id, total);
    } else {
        return MakeAtomicShell(id, total); 
    }
}

TrajectoryFunc GetTargetTrajectory() {
    if (SCENARIO_TYPE == 3) {
        // Nello scenario 3, il drone 0 è parte della formazione geometrica
        return MakeOctahedronFormation(0); 
    } else {
        return MakeFigureEightTarget();
    }
}
