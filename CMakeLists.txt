# CMakeLists.txt per simulazione TDOA UWB

# 1. Cerca Eigen (necessario per calcoli matriciali EKF)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})

# 2. Definisci l'eseguibile
# NOTA: CentralProcessor.cpp è stato rimosso. 
# SimulationLogger.h è header-only, quindi non serve elencarlo qui.
add_executable(tdoa-uwb-run
    tdoa_uwb_sim.cpp
    Trajectories.cpp
    TDoAEKF.cpp
    UWBChannel.cpp
    Drone.cpp
)

# 3. Collega le librerie necessarie di NS-3
target_link_libraries(tdoa-uwb-run
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libinternet}
    ${libapplications}
    ${libwifi}
    ${libspectrum}
    ${libpropagation}
    Eigen3::Eigen
)
