#include <fstream>
#include <chrono>
#include <iostream>
#include <cmath>
#include <random>
#include "mlat_core.h"

using namespace Eigen;
using namespace std;

int main() {
    ofstream resFile("mlat_results_din.csv");
    resFile << "tx,ty,tz,ex,ey,ez\n";

    ofstream ancFile("anchors_trace.csv");
    ancFile << "ax0,ay0,az0,ax1,ay1,az1,ax2,ay2,az2,ax3,ay3,az3,ax4,ay4,az4\n";

    Vector3d last_pos(0,0,0);
    bool has_last = false;
    double R = 35.0; 

    double total_time_accumulated = 0;
    int valid_steps = 0;

	std::random_device rd;
	std::mt19937 gen(rd());
	std::normal_distribution<double> dist_noise(0.0, 0.2);
	
    for (int i = 0; i < 400; i++) 
    {
        double t = i * (2 * M_PI / 400.0); 
        vector<Vector3d> anchors;
        anchors.push_back({ R*cos(t), R*sin(t), 15 });
		anchors.push_back({ 20*cos(t), 20*sin(t), 15 + 15*sin(t*2) });
		anchors.push_back({ R*cos(t)*sin(2*t), R*sin(t)*sin(2*t), 25 });
        anchors.push_back({ 25*cos(t*1.5), 0, 30 + 20*sin(t*1.5) });
        anchors.push_back({ 0, 25*cos(t*2.0), 30 + 20*sin(t*2.0) });

        for (size_t k=0; k<anchors.size(); k++) {
            ancFile << anchors[k](0) << "," << anchors[k](1) << "," << anchors[k](2);
            if (k < anchors.size()-1) ancFile << ",";
        }
        ancFile << "\n";
        Vector3d target_pos(-30.0 + 60.0*cos(t*0.5), -30.0 + 60.0*sin(t*0.5), 5.0 + i*0.1);
        vector<double> times;
        for (size_t i = 0; i < anchors.size(); ++i) {
            const auto& anc = anchors[i];
            double dist = (anc - target_pos).norm();
            double linear_noise = ((rand() % 100) / 100.0 - 0.5) * 0.2;
            double gaussian_noise = dist_noise(gen);
            times.push_back(dist + linear_noise);
        }
        

        auto start_timer = chrono::high_resolution_clock::now();
        auto result = mlat_solver(anchors, times, last_pos, has_last);
        auto stop_timer = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(stop_timer - start_timer).count();
        
        if(result.first) 
        {
            last_pos = result.second; 
            has_last = true;
            total_time_accumulated += duration;
            valid_steps++;
            
            resFile << target_pos(0) << "," << target_pos(1) << "," << target_pos(2) << ","
                    << result.second(0) << "," << result.second(1) << "," << result.second(2) << "\n";
        }
    }

    resFile.close(); ancFile.close();
    cout << "Simulazione completata." << endl;
    if (valid_steps > 0) 
    {
        cout << "Tempo Medio per calcolo: " << (total_time_accumulated / valid_steps) << " microsecondi (us)" << endl;
    }
    return 0;
}
