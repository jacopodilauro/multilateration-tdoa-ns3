
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys

def main():
    filename = 'tdma_security_log.csv'
    print(f"--- Dashboard Analisi Drone (Ottimizzata) ---")
    
    try:
        df = pd.read_csv(filename)
    except FileNotFoundError:
        print(f"ERRORE: File '{filename}' non trovato.")
        return

    available_ids = sorted(df['sender_id'].unique())
    print(f"ID Trovati: {available_ids}")

    target_id = 0 # Default
    if len(sys.argv) > 1:
        try:
            target_id = int(sys.argv[1])
        except ValueError: pass
    else:
        try:
            user_input = input(f"Inserisci ID target per analisi (Default 0): ")
            if user_input.strip(): target_id = int(user_input)
        except ValueError: pass

    df_target = df[df['sender_id'] == target_id]
    if df_target.empty:
        print("Nessun dato trovato per questo target.")
        return

    # Creazione della figura principale con layout a griglia
    fig = plt.figure(figsize=(15, 10))
    fig.canvas.manager.set_window_title(f'Analisi Avanzata Target {target_id}')
    
    # --- 1. VISIONE GLOBALE 3D (Top Left) ---
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.set_title("1. Traiettorie Reali Sciame (Ground Truth)", fontsize=12, pad=20)
    
    colors = plt.cm.jet(np.linspace(0, 1, len(available_ids)))
    for i, drone_id in enumerate(available_ids):
        traj = df[df['sender_id'] == drone_id].drop_duplicates(subset=['time'])
        if traj.empty: continue
        c = colors[i]
        ax1.plot(traj['true_x'], traj['true_y'], traj['true_z'], color=c, label=f'D{drone_id}', alpha=0.7)
        end = traj.iloc[-1]
        ax1.scatter(end['true_x'], end['true_y'], end['true_z'], color=c, marker='s')
        ax1.text(end['true_x'], end['true_y'], end['true_z'], f"D{drone_id}", fontsize=9)

    ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')
    ax1.legend(fontsize='x-small', loc='upper left')

    # --- 2. ANALISI COMPARATIVA 3D (Top Right) ---
    ax2 = fig.add_subplot(2, 2, 2, projection='3d')
    ax2.set_title(f"2. Confronto Stime sul Target {target_id}", fontsize=12, pad=20)

    truth_traj = df_target.drop_duplicates(subset=['time'])
    ax2.plot(truth_traj['true_x'], truth_traj['true_y'], truth_traj['true_z'], 
             color='black', linewidth=3, label='REALTÀ')

    if not df_target.empty:
        sample_obs = df_target['observer_id'].iloc[0]
        claim_traj = df_target[df_target['observer_id'] == sample_obs]
        ax2.plot(claim_traj['claim_x'], claim_traj['claim_y'], claim_traj['claim_z'], 
                 color='red', linewidth=1.5, linestyle='--', label='GPS DICHIARATO')

    observer_ids = sorted(df_target['observer_id'].unique())
    obs_colors = plt.cm.tab10(np.linspace(0, 1, len(observer_ids)))

    for i, obs_id in enumerate(observer_ids):
        data = df_target[df_target['observer_id'] == obs_id]
        if data.empty: continue
        ax2.plot(data['est_x'], data['est_y'], data['est_z'], 
                 color=obs_colors[i], linewidth=1, alpha=0.8, label=f'Stima da D{obs_id}')

    ax2.set_xlabel('X'); ax2.set_ylabel('Y'); ax2.set_zlabel('Z')
    ax2.legend(fontsize='x-small', loc='upper right')

    # --- 3. ANALISI ERRORI 2D (Bottom - Full Width) ---
    ax3 = fig.add_subplot(2, 1, 2)
    ax3.set_title(f"3. Evoluzione Errore Euclideo per Target {target_id}", fontsize=12)
    
    for i, obs_id in enumerate(observer_ids):
        data = df_target[df_target['observer_id'] == obs_id]
        if data.empty: continue
        ax3.plot(data['time'], data['estimation_error'], 
                 label=f'Errore Obs D{obs_id}', linewidth=1.5)

    ax3.set_xlabel('Tempo [s]')
    ax3.set_ylabel('Errore [m]')
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.legend(fontsize='small', ncol=min(len(observer_ids), 4))

    plt.tight_layout()
    print("Grafici generati in un'unica dashboard.")
    plt.show()

if __name__ == "__main__":
    main()
