<h1 align="center">📡 TDoA Multilateration for UWB Swarm Simulation & Security</h1>

A high-fidelity simulation environment based on **ns-3** designed to study: **Multilateration, Time Difference of Arrival (TDoA)** localization, clock synchronization, and security mechanisms within dynamic drone swarms.

This project implements a distributed **Extended Kalman Filter (EKF)** to estimate the positions of neighboring drones using Ultra-Wideband (UWB) ranging frames. It also features a security layer capable of detecting **GPS Spoofing attacks** by analyzing the discrepancy between claimed GPS positions and the physics-based TDoA estimates.

---

## Key Features

* **Realistic UWB Channel Model**: Custom implementation (`UWBChannel`) accounting for:
    * Path Loss & Shadowing (Log-distance model).
    * Line-of-Sight (LOS) vs Non-Line-of-Sight (NLOS) probability.
    * Ranging errors and delay spread based on environmental conditions.
* **Distributed State Estimation**:
    * Each drone runs a local bank of **Extended Kalman Filters (EKF)**.
    * Estimates neighbor positions (X, Y, Z) and clock bias in real-time.
* **Clock Synchronization**:
    * Simulates hardware clock drift (physical layer).
    * Implements a software-based synchronization algorithm using a Master Anchor approach to correct offsets.
* **Security & Threat Simulation**:
    * **Attack Simulation**: Ability to inject malicious behavior (e.g., GPS spoofing where a drone lies about its position).
    * **Anomaly Detection**: Automated alarm system that triggers when the "Discrepancy" (residual) between the claimed GPS position and the TDoA estimate exceeds a safety threshold.
* **Dynamic 3D Trajectories**:
    * Supports complex formation flying scenarios (e.g., Octahedron Swarm, Atomic Shell, Figure-8).

---

## Dependencies

To build and run this simulation, you need the following installed on your system:

* **[ns-3](https://www.nsnam.org/)** (Network Simulator 3)
    * Required modules: `core`, `network`, `mobility`, `wifi`, `applications`.
* **[Eigen3](https://eigen.tuxfamily.org/)**: For matrix operations in the EKF.
* **CMake** (version 3.3 or higher).
* **Python 3** (for data analysis and plotting).
    * Libraries: `pandas`, `matplotlib`, `numpy`.

---

## Installation & Build

1.  **Clone the repository**:
    ```bash
    git clone https://github.com/jacopodilauro/mlat-dynamic
    ```

2.  Using ns3
    ```bash
    cd ns3/ns-3-allinone/ns-3.46.1
    ```

3.  **Configure with CMake**:
    Ensure `ns-3` libraries are visible to CMake (you may need to adjust paths depending on your ns-3 installation).
    ```bash
    ./ns3 build
    ```

4.  **Compile**:
    ```bash
    ./build/tdoa-uwb-run
    ```
    
If you wnat to see the result in graphic mode, do:
5.  **python**:
   ```bash
     python3 scratch/4.5_solo_real/plot_tdoa.py
  ```
---
