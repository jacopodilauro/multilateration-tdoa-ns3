#include "mlat_core.h"
#include <iostream>
#include <cmath>
#include <random>

using namespace Eigen;
using namespace std;

Vector3d select_best_solution(const Vector3d& P1, const Vector3d& P2, const Vector3d& last_pos, bool has_last) 
{
	double z_min = -5.0;
	bool P1_valida = P1.z() >= z_min;
	bool P2_valida = P2.z() >= z_min;
	if (P1_valida && !P2_valida) return P1;
	if (P2_valida && !P1_valida) return P2;
	if (has_last) {
		double dist1 = (P1 - last_pos).norm();
		double dist2 = (P2 - last_pos).norm();
		return (dist1 < dist2) ? P1 : P2;
	}
	return P1;
}


pair<bool, Vector3d> mlat_solver(const vector<Vector3d>& anchors, const vector<double>& times, const Vector3d& last_pos, bool has_last) {
    if (anchors.size() < 4) return { false, Vector3d::Zero() };

    Vector3d ref = anchors[0];
    double ref_t = times[0];

    if (anchors.size() >= 5) 
    {
        int n_eq = anchors.size() - 1;

        MatrixXd A(n_eq, 4);
        VectorXd b(n_eq);

        for (int i = 1; i < anchors.size(); i++) 
        {
            double dd = times[i] - ref_t;
            A.row(i-1).head(3) = 2.0 * (ref - anchors[i]);
            A(i-1, 3) = -2.0 * dd;
            b(i-1) = pow(dd, 2) - (anchors[i].squaredNorm() - ref.squaredNorm());
        }
        Vector4d result = A.colPivHouseholderQr().solve(b);

        return {true, result.head(3)};
    }else
    {
	MatrixXd A(3, 3); 
	VectorXd b(3), v(3);
	for (int i=1; i<4; i++) 
	{
	        double dd = times[i] - ref_t;
	        A.row(i-1) = 2.0 * (ref - anchors[i]);
	        b(i-1) = pow(dd, 2) - (anchors[i].squaredNorm() - ref.squaredNorm());
	        v(i-1) = 2.0 * dd;
	}
    
    	auto dec = A.colPivHouseholderQr();
	Vector3d S = dec.solve(b);
	Vector3d Q = dec.solve(v);
    	Vector3d D = S - ref;
    	
    	double a = Q.dot(Q)-1.0, 
    	       b_c = 2.0*Q.dot(D), 
    	       c = D.dot(D);
    	double delta = b_c*b_c - 4*a*c;
    	if (delta < 0) return {false, Vector3d::Zero()};
    	Vector3d P1 = S + ((-b_c + sqrt(delta))/(2*a))*Q;
	Vector3d P2 = S + ((-b_c - sqrt(delta))/(2*a))*Q;
	
    	return { true, select_best_solution(P1, P2, last_pos, has_last) };
    }
}
