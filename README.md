# Instructions for MLAT Project Execution

Follow the commands below in your console, in the specified order. You must have **Eigen** and a **C++ compiler** installed.

---

### C++ Code Compilation

Use `g++` to compile the C++ source files (`mlatcpp_din5.cpp` and `mlat_core.cpp`) and create the executable file named `mlat_ppdin5`.

Ensure that the include path for **Eigen3** is correct for your system.

```bash
g++ -I /usr/include/eigen3 mlatcpp_din5.cpp mlat_core.cpp -o mlat_ppdin5
```
dopo
```bash
./mlat_ppdin5
```
```bash
python mlatpy_din5.py
```
