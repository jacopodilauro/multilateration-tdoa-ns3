import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys

# Impostazioni stile per pubblicazioni (font più leggibili)
plt.rcParams.update({'font.size': 12, 'figure.autolayout': True})

def calculate_metrics(df_target):
    """Calcola metriche aggregate per tabelle LaTeX/Paper"""
    rmse = np.sqrt((df_target['estimation_error'] ** 2).mean())
    max_err = df_target['estimation_error'].max()
    mean_err = df_target['estimation_error'].mean()
    
    # Rilevamento falsi positivi/negativi richiede ground truth logica, 
    # qui calcoliamo la % di tempo in allarme
    total_samples = len(df_target)
    alarm_samples = df_target['alarm'].sum()
    alarm_rate = (alarm_samples / total_samples) * 100 if total_samples > 0 else 0
    
    return mean_err, rmse, max_err, alarm_rate

def plot_error_analysis(df_target, target_id, observer_ids):
    """Genera grafici 2D per analisi quantitativa"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title(f'4. Analisi Errori & Sicurezza Target {target_id}')
    
    colors = plt.cm.tab10(np.linspace(0, 1, len(observer_ids)))

    # --- SUBPLOT 1: ERRORE DI STIMA (Performance) ---
    for i, obs_id in enumerate(observer_ids):
        data = df_target[df_target['observer_id'] == obs_id]
        if data.empty: continue
        
        # Plot Errore
        ax1.plot(data['time'], data['estimation_error'], 
                 label=f'Obs {obs_id}', color=colors[i], linewidth=1.5, alpha=0.8)

    ax1.set_ylabel('Errore Posizione [m]')
    ax1.set_title('Accuratezza Localizzazione (Ground Truth vs EKF)', fontsize=14)
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend(loc='upper right', fontsize='small', ncol=2)

    # --- SUBPLOT 2: DISCREPANZA & ALLARMI (Sicurezza) ---
    for i, obs_id in enumerate(observer_ids):
        data = df_target[df_target['observer_id'] == obs_id]
        if data.empty: continue
        
        # Linea della Discrepanza (Residuo)
        ax2.plot(data['time'], data['discrepancy'], 
                 color=colors[i], linewidth=1.5, alpha=0.8, label=f'Residuo Obs {obs_id}')
        
        # --- EVIDENZIAZIONE ALLARME ---
        # Trova gli intervalli dove l'allarme è attivo
        alarm_active = data[data['alarm'] == 1]
        if not alarm_active.empty:
            # Usiamo fill_between o bar per evidenziare le zone di allarme
            # Trucco: plotto barre verticali rosse con alpha basso dove c'è allarme
            # Per evitare sovrapposizioni pesanti, lo facciamo solo per il primo o cumulativo?
            # Qui lo facciamo per ogni osservatore leggermente shiftato o unito
            ax2.scatter(alarm_active['time'], [0]*len(alarm_active), 
                        color='red', marker='|', s=100, alpha=0.5, label='_nolegend_')

    # Soglia di allarme (hardcoded a 5.0 come nel C++)
    ax2.axhline(y=5.0, color='r', linestyle='--', linewidth=2, label='Soglia Allarme (5m)')
    
    # Evidenzia zone di attacco (sfondo rosso) se l'allarme è attivo per la maggioranza
    # Calcolo un "voto di maggioranza" per lo sfondo
    times = df_target['time'].unique()
    consensus_alarm = []
    for t in times:
        active_alarms = df_target[df_target['time'] == t]['alarm'].sum()
        if active_alarms >= len(observer_ids) / 2: # Maggioranza
            consensus_alarm.append(t)
    
    if consensus_alarm:
        # Disegna bande rosse verticali
        for t in consensus_alarm:
            ax2.axvspan(t - 0.05, t + 0.05, color='red', alpha=0.05)
        # Etichetta solo una volta
        ax2.text(consensus_alarm[0], ax2.get_ylim()[1]*0.9, " ALARM ACTIVE", color='red', fontweight='bold')

    ax2.set_ylabel('Discrepanza (Claim vs Stima) [m]')
    ax2.set_xlabel('Tempo Simulazione [s]')
    ax2.set_title(r'Rilevamento Anomalie (Residuo > Soglia)', fontsize=14)
    ax2.grid(True, linestyle='--', alpha=0.6)
    
    plt.tight_layout()

def main():
    filename = 'tdma_security_log.csv'
    print(f"--- Generatore Grafici per Paper (TDoA UWB) ---")
    
    try:
        df = pd.read_csv(filename)
    except FileNotFoundError:
        print(f"ERRORE: File '{filename}' non trovato.")
        return

    available_ids = sorted(df['sender_id'].unique())
    
    # Selezione Target
    target_id = 0 
    if len(sys.argv) > 1:
        target_id = int(sys.argv[1])
    
    df_target = df[df['sender_id'] == target_id]
    if df_target.empty:
        print("Nessun dato per il target.")
        return

    observer_ids = sorted(df_target['observer_id'].unique())
    
    # 1. Calcolo Statistiche
    mean_e, rmse, max_e, alarm_perc = calculate_metrics(df_target)
    print(f"\n--- STATISTICHE TARGET {target_id} (per LaTeX) ---")
    print(f"Mean Error: {mean_e:.4f} m")
    print(f"RMSE:       {rmse:.4f} m")
    print(f"Max Error:  {max_e:.4f} m")
    print(f"Alarm Active Time: {alarm_perc:.2f}%")
    print("--------------------------------------------------\n")

    # 2. Grafico Errori & Allarmi (NUOVO)
    plot_error_analysis(df_target, target_id, observer_ids)

    # 3. Grafico 3D Comparativo (RIDOTTO/PULITO per Paper)
    fig_comp = plt.figure(figsize=(10, 8))
    fig_comp.canvas.manager.set_window_title(f'3. Traiettorie 3D Paper')
    ax_comp = fig_comp.add_subplot(111, projection='3d')
    
    # Ground Truth (Nero Solido)
    truth_traj = df_target.drop_duplicates(subset=['time'])
    ax_comp.plot(truth_traj['true_x'], truth_traj['true_y'], truth_traj['true_z'], 
                 color='k', linewidth=2, label='Ground Truth')
    
    # Claimed (Rosso Tratteggiato - L'attacco)
    # Prendiamo un observer a caso per vedere cosa dichiara il target
    if not df_target.empty:
        sample_obs = df_target['observer_id'].iloc[0]
        claim_traj = df_target[df_target['observer_id'] == sample_obs]
        ax_comp.plot(claim_traj['claim_x'], claim_traj['claim_y'], claim_traj['claim_z'], 
                     color='r', linestyle='--', linewidth=2, label='Spoofed Path')

    # Estimated (Blu/Verde) - Plottiamo solo la media o uno rappresentativo per pulizia
    # Qui plottiamo il Drone 2 come esempio
    example_obs = observer_ids[0] if len(observer_ids) > 0 else 0
    est_data = df_target[df_target['observer_id'] == example_obs]
    if not est_data.empty:
         ax_comp.plot(est_data['est_x'], est_data['est_y'], est_data['est_z'], 
                     color='b', linewidth=1, alpha=0.7, label=f'EKF Estimate (Obs {example_obs})')

    ax_comp.set_xlabel('X [m]')
    ax_comp.set_ylabel('Y [m]')
    ax_comp.set_zlabel('Z [m]')
    ax_comp.legend()
    ax_comp.set_title(f"Ricostruzione Traiettoria vs Attacco (Target {target_id})")

    print("Visualizzazione grafici...")
    plt.show()

if __name__ == "__main__":
    main()
