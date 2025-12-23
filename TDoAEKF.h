#ifndef TDOAEKF_H
#define TDOAEKF_H

#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

class TDoAEKF {
public:
    TDoAEKF();
    
    void Init(const Vector3d& init_pos);

    void Predict(double dt);

    struct Msmnt {
        Vector3d anchor_pos;
        double toa;
        double tx_timestamp;
    };
    void Update(const vector<Msmnt>& measurements);

    Vector3d GetPosition() const;
    VectorXd GetState() const;

private:
    VectorXd m_state; // Dimensione 7
    MatrixXd m_P;     // Covarianza 7x7
    MatrixXd m_Q;     // Rumore di processo 7x7
    
    const double c = 299792458.0; 
};

#endif
