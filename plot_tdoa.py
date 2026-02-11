import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys

# Impostazioni grafiche generali
plt.rcParams.update({'font.size': 10, 'figure.autolayout': True})

def calculate_metrics(df_target):
    rmse = np.sqrt((df_target['estimation_error'] ** 2).mean())
    max_err = df_target['estimation_error'].max()
    mean_err = df_target['estimation_error'].mean()
    
    total_samples = len(df_target)
    alarm_samples = df_target['alarm'].sum()
    alarm_rate = (alarm_samples / total_samples) * 100 if total_samples > 0 else 0
    
    return mean_err, rmse, max_err, alarm_rate

def main():
    filename = 'tdma_security_log.csv'
    print(f"--- Dashboard Unificata TDoA UWB ---")
    
    try:
        df = pd.read_csv(filename)
    except FileNotFoundError:
        print(f"ERRORE: File '{filename}' non trovato. Esegui prima la simulazione ns-3.")
        return

    # Selezione Target (Default 0)
    target_id = 0 
    if len(sys.argv) > 1:
        try:
            target_id = int(sys.argv[1])
        except ValueError: pass
    
    df_target = df[df['sender_id'] == target_id]
    if df_target.empty:
        print(f"Nessun dato trovato per il target {target_id}.")
        return

    observer_ids = sorted(df_target['observer_id'].unique())
    
    # Calcolo statistiche
    mean_e, rmse, max_e, alarm_perc = calculate_metrics(df_target)
    print(f"\n--- STATISTICHE TARGET {target_id} ---")
    print(f"Mean Error: {mean_e:.4f} m")
    print(f"Alarm Active: {alarm_perc:.2f}%")
    
    # --- CREAZIONE FIGURA UNIFICATA ---
    fig = plt.figure(figsize=(16, 8))
    fig.suptitle(f"Analisi Completa Sicurezza & Traiettoria (Target {target_id})", fontsize=16)
    
    # Definizione Layout: 2 righe, 2 colonne
    # La colonna di sinistra (0) sarà larga il doppio della destra (width_ratios)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.3, 1])

    # ---------------------------------------------------------
    # 1. GRAFICO 3D (Colonna Sinistra, occupa entrambe le righe)
    # ---------------------------------------------------------
    ax_3d = fig.add_subplot(gs[:, 0], projection='3d')
    
    truth_traj = df_target.drop_duplicates(subset=['time'])
    ax_3d.plot(truth_traj['true_x'], truth_traj['true_y'], truth_traj['true_z'], 
               color='k', linewidth=2.5, label='Realtà (Ground Truth)')
    
    # Traiettoria Spoofata (dal punto di vista del primo osservatore, se presente)
    if not df_target.empty:
        sample_obs = df_target['observer_id'].iloc[0]
        claim_traj = df_target[df_target['observer_id'] == sample_obs]
        ax_3d.plot(claim_traj['claim_x'], claim_traj['claim_y'], claim_traj['claim_z'], 
                   color='r', linestyle='--', linewidth=2, label='GPS Dichiarato (Spoofing)')

    # Stima EKF (di un osservatore a caso per pulizia)
    example_obs = observer_ids[0] if len(observer_ids) > 0 else 0
    est_data = df_target[df_target['observer_id'] == example_obs]
    if not est_data.empty:
         ax_3d.plot(est_data['est_x'], est_data['est_y'], est_data['est_z'], 
                    color='b', linewidth=1, alpha=0.6, label=f'Stima EKF (Obs {example_obs})')

    ax_3d.set_xlabel('X [m]')
    ax_3d.set_ylabel('Y [m]')
    ax_3d.set_zlabel('Z [m]')
    ax_3d.set_title("Ricostruzione 3D vs Attacco")
    ax_3d.legend(loc='upper right')

    # ---------------------------------------------------------
    # 2. GRAFICO ERRORI (Colonna Destra, Riga Superiore)
    # ---------------------------------------------------------
    ax_err = fig.add_subplot(gs[0, 1])
    colors = plt.cm.tab10(np.linspace(0, 1, len(observer_ids)))

    for i, obs_id in enumerate(observer_ids):
        data = df_target[df_target['observer_id'] == obs_id]
        if data.empty: continue
        ax_err.plot(data['time'], data['estimation_error'], 
                 label=f'Obs {obs_id}', color=colors[i], linewidth=1.5, alpha=0.8)
    
    ax_err.set_title('Accuratezza Localizzazione (Ground Truth vs EKF)')
    ax_err.set_ylabel('Errore Posizione [m]')
    ax_err.grid(True, linestyle='--', alpha=0.6)
    ax_err.legend(fontsize='x-small', loc='upper right', ncol=2)

# ---------------------------------------------------------
    # 3. GRAFICO ALLARMI (Colonna Destra, Riga Inferiore)
    # ---------------------------------------------------------
    ax_alm = fig.add_subplot(gs[1, 1], sharex=ax_err)
    
    for i, obs_id in enumerate(observer_ids):
        data = df_target[df_target['observer_id'] == obs_id]
        if data.empty: continue
        ax_alm.plot(data['time'], data['discrepancy'], 
                 color=colors[i], linewidth=1.0, alpha=0.6)

    # Linea di soglia
    ax_alm.axhline(y=10.0, color='r', linestyle='--', linewidth=2, label='Soglia (10m)')
    
    # --- CORREZIONE LOGICA AREA ROSSA ---
    # Calcoliamo il consenso istante per istante
    unique_times = sorted(df_target['time'].unique())
    consensus_bool = []
    
    for t in unique_times:
        # Conta quanti droni hanno l'allarme attivo in questo istante
        votes = df_target[df_target['time'] == t]['alarm'].sum()
        # Se più della metà dei droni vota allarme, è un allarme globale
        consensus_bool.append(votes >= len(observer_ids) / 2)
    
    # fill_between con 'where' gestisce correttamente i buchi (zone sicure)
    # transform=ax_alm.get_xaxis_transform() ci permette di colorare tutta l'altezza (da 0 a 1 relativo all'asse)
    ax_alm.fill_between(unique_times, 0, 1, where=consensus_bool, 
                        color='red', alpha=0.2, transform=ax_alm.get_xaxis_transform())
    
    # Testo indicativo (posizionato approssimativamente dove inizia l'attacco vero)
    ax_alm.text(210, ax_alm.get_ylim()[1]*0.85, " ALARM ACTIVE", 
                color='red', fontweight='bold', fontsize=9)

    ax_alm.set_title('Rilevamento Anomalie (Residuo > Soglia)')
    ax_alm.set_xlabel('Tempo Simulazione [s]')
    ax_alm.set_ylabel('Discrepanza [m]')
    ax_alm.grid(True, linestyle='--', alpha=0.6)

    plt.show()

if __name__ == "__main__":
    main()