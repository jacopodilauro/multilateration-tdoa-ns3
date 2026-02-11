<h1 align="center">📡 TDoA Multilateration for Drone Swarm</h1>

<p align="center">
   Ns-3 simulation for distributed localization, clock synchronization, and GPS spoofing detection.
</p>

<p align="center">
  <img width="60%" alt="Confronto Stime Target 0" src="images/img_traject2.png" />
</p>

---

## Overview

This project implements **ns-3 simulation** designed to model drone swarm using Ultra-WideBand (UWB) tecnology for GPS spoofing detection. The system uses Time Difference of Arrival (TDoA) multilateration. 
It's compose into this classes:
1. **tdoa_main.cpp**: inizialize the swarm's initialization and Round Robin TDMA scheduler. It manages the simulation timeline, including the activation of a GPS spoofing attack on a target drone at a specific time (t=200s).
2. **Drone.cpp/h & .h**: Defines the behavior of each swarm member, including its true position, noisy GPS reporting, and clock drift management.
3. **UWBMessage.h**: compose the data structure for comunication
4. **TDoAEKF.cpp/h**: Custom Distributed Extended Kalman Filter (EKF).
5. **UWBChannel.cpp/h**: Simulates a custom UWB physical layer based on IEEE 802.15.4a standards, it include Line-of-Sight (LOS) and Non-Line-of-Sight (NLOS) conditions, path loss and distance error.
6. **Trajectories.cpp/h**: Provides mathematical functions for various flight patterns, such as Atomic Shell, Circular Patrol, and **Octahedron formations**. 



---

## Dependencies

To build and run this simulation, ensure you have the following installed:

* **[ns-3](https://www.nsnam.org/)** (v3.30+ recommended)
    * Modules: `core`, `network`, `mobility`.
* **[Eigen3](https://eigen.tuxfamily.org/)**: Required for EKF matrix operations.
* **CMake** (v3.3+).
* **Python 3** (Matplotlib, Pandas, NumPy) for plotting.

---

## Installation & Usage

1.  **Clone the repository in `ns-3-allinone/ns-3.46.1/scratch`**:
    ```bash
    git clone https://github.com/jacopodilauro/multilateration-tdoa-ns3
    ```

2.  **Move to ns-3 directory**:
    Assuming you have ns-3 installed:
    ```bash
    cd ns-3-allinone/ns-3.46.1/
    ```

3.  **Configure & Build**:
    ```bash
    ./ns3 configure --enable-examples --enable-tests
    ./ns3 build
    ```

4.  **Run the Simulation**:
    ```bash
    ./build/tdma_main
    ```
    *This generates a `tdma_security_log.csv` file containing the telemetry.*

5.  **Visualize Results**:
    Use the provided Python script to generate the 3D plots and error analysis:
    ```bash
    python3 scratch/plot_tdoa.py
    python3 scratch/plot_old.py
    ```

---

## 📄 License

Distributed under the MIT License. See `LICENSE` for more information.

