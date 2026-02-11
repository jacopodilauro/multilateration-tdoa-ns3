#include "TDoAEKF.h"
#include <iostream>

using namespace Eigen;
using namespace std;

TDoAEKF::TDoAEKF() {
    m_state = VectorXd::Zero(7);
    m_P = MatrixXd::Identity(7, 7);
    m_Q = MatrixXd::Identity(7, 7);
}

void TDoAEKF::Init(const Vector3d& init_pos) {
    m_state = VectorXd::Zero(7);
    m_state.segment<3>(0) = init_pos; 
    m_state(6) = 0.0; 
    
    m_P = MatrixXd::Identity(7, 7);
    m_P.block<3,3>(0,0) *= 5.0; 
    m_P.block<3,3>(3,3) *= 1.0;  
    m_P(6,6) = 500.0;            
    
    m_Q = MatrixXd::Identity(7, 7) * 1; 
    m_Q(6,6) = 1.0; 
}

void TDoAEKF::Predict(double dt) {
    if (dt <= 0) return;
    
    MatrixXd F = MatrixXd::Identity(7, 7);
    F(0, 3) = dt; 
    F(1, 4) = dt; 
    F(2, 5) = dt; 
    
    m_state = F * m_state;
    m_P = F * m_P * F.transpose() + m_Q;
}

void TDoAEKF::Update(const vector<Msmnt>& measurements) {
    if (measurements.empty()) return;

    int n = measurements.size();
    VectorXd Z(n);       
    VectorXd h(n);       
    MatrixXd H(n, 7); 
    
    MatrixXd R = MatrixXd::Identity(n, n) * 2.0; 

    Vector3d est_pos = m_state.segment<3>(0); 
    double est_bias = m_state(6); 

    for(int i = 0; i < n; ++i) {
        Vector3d anchor_pos = measurements[i].anchor_pos;
        
        double pseudorange = (measurements[i].toa - measurements[i].tx_timestamp) * c;
        Z(i) = pseudorange;
        double geo_dist = (est_pos - anchor_pos).norm();
        h(i) = geo_dist + est_bias;
        double safe_dist = geo_dist + 1e-9;
        H(i, 0) = (est_pos.x() - anchor_pos.x()) / safe_dist;
        H(i, 1) = (est_pos.y() - anchor_pos.y()) / safe_dist;
        H(i, 2) = (est_pos.z() - anchor_pos.z()) / safe_dist;
        H(i, 3) = 0; H(i, 4) = 0; H(i, 5) = 0;
        H(i, 6) = 1.0; 
    }

    VectorXd y = Z - h; 
    MatrixXd S = H * m_P * H.transpose() + R;
    MatrixXd K = m_P * H.transpose() * S.inverse(); 
    
    m_state = m_state + K * y;
    m_P = (MatrixXd::Identity(7, 7) - K * H) * m_P;
}

Vector3d TDoAEKF::GetPosition() const { return m_state.segment<3>(0); }
VectorXd TDoAEKF::GetState() const { return m_state; }
