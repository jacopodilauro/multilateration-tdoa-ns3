<h1 align="center">📡 TDoA Multilateration for UWB Swarm Simulation & Security</h1>

<p align="center">
  <strong>High-fidelity ns-3 simulation for distributed localization, clock synchronization, and GPS spoofing detection.</strong>
</p>

<p align="center">
  <img width="60%" alt="Confronto Stime Target 0" src="images/img_traject2.png" />
</p>

---

## Overview
This project implements in **ns-3** a swarm of drones that uses **Ultra-Wideband (UWB)** technology for mutual localization.
The goal is security: by comparing the GPS position declared by each drone with the physical TDoA measurement, the system is able to isolate compromised nodes that transmit false coordinates.

The heart of the communication is managed via a **TDMA Round Robin** protocol, which guarantees order and determinism in the exchange of ranging messages and an **Extended Kalman Filter (EKF)** for the misurement.

---

## Dependencies

To build and run this simulation, ensure you have the following installed:

* **[ns-3](https://www.nsnam.org/)** (v3.30+ recommended)
    * Modules: `core`, `network`, `mobility`, `wifi`, `applications`.
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
    ./build/tdoa-uwb-run
    ```
    *This generates a `tdma_security_log.csv` file containing the telemetry.*

5.  **Visualize Results**:
    Use the provided Python script to generate the 3D plots and error analysis:
    ```bash
    ~/ns-3.46.1$ python3 scratch/multilateration-tdoa-ns3/plot_tdoa.py
    ~/ns-3.46.1$ python3 scratch/multilateration-tdoa-ns3/plot_old.py
    ~/ns-3.46.1$ python3 scratch/multilateration-tdoa-ns3/test_swarm_voting
    ```

---

## 📄 License

Distributed under the MIT License. See `LICENSE` for more information.

