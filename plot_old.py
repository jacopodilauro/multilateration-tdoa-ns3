import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys

def main():
    filename = 'tdma_security_log.csv'
    print(f"--- Visualizzatore Sciame (Solo Visione Globale e Dettagli) ---")
    
    try:
        df = pd.read_csv(filename)
    except FileNotFoundError:
        print(f"ERRORE: File '{filename}' non trovato.")
        return

    available_ids = sorted(df['sender_id'].unique())
    print(f"ID Trovati: {available_ids}")

    target_id = 0
    if len(sys.argv) > 1:
        try:
            target_id = int(sys.argv[1])
        except ValueError: pass
    else:
        try:
            user_input = input(f"Inserisci ID target per analisi dettaglio (Default 0): ")
            if user_input.strip(): target_id = int(user_input)
        except ValueError: pass

    df_target = df[df['sender_id'] == target_id]

    if df_target.empty:
        print("Nessun dato trovato per questo target.")
        return

    # ---------------------------------------------------------
    # 1. VISIONE GLOBALE (Tutti i droni insieme)
    # ---------------------------------------------------------
    fig_global = plt.figure(figsize=(10, 8))
    fig_global.canvas.manager.set_window_title('1. Visione Globale Sciame')
    ax_global = fig_global.add_subplot(111, projection='3d')
    ax_global.set_title("Traiettorie Reali (Ground Truth)", fontsize=15)

    colors = plt.cm.jet(np.linspace(0, 1, len(available_ids)))

    for i, drone_id in enumerate(available_ids):
        traj = df[df['sender_id'] == drone_id].drop_duplicates(subset=['time'])
        if traj.empty: continue
        c = colors[i]
        ax_global.plot(traj['true_x'], traj['true_y'], traj['true_z'], color=c, label=f'D{drone_id}')
        
        # Segna inizio e fine
        start = traj.iloc[0]; end = traj.iloc[-1]
        ax_global.scatter(start['true_x'], start['true_y'], start['true_z'], color=c, marker='o', facecolors='none')
        ax_global.scatter(end['true_x'], end['true_y'], end['true_z'], color=c, marker='s')
        ax_global.text(end['true_x'], end['true_y'], end['true_z'], f"D{drone_id}", color='black')

    ax_global.legend()
    ax_global.set_xlabel('X')
    ax_global.set_ylabel('Y')
    ax_global.set_zlabel('Z')
    
    # Scaling assi per mantenere le proporzioni
    all_x = df['true_x']; all_y = df['true_y']; all_z = df['true_z']
    mid_x = (all_x.max()+all_x.min())/2; mid_y = (all_y.max()+all_y.min())/2; mid_z = (all_z.max()+all_z.min())/2
    max_range = max(all_x.max()-all_x.min(), all_y.max()-all_y.min(), all_z.max()-all_z.min()) / 2
    ax_global.set_xlim(mid_x-max_range, mid_x+max_range)
    ax_global.set_ylim(mid_y-max_range, mid_y+max_range)
    ax_global.set_zlim(mid_z-max_range, mid_z+max_range)

    observer_ids = sorted(df['observer_id'].unique())
    num_obs = len(observer_ids)
    
    # ---------------------------------------------------------
    # 2. DETTAGLIO TARGET (Cosa vede ogni singolo drone)
    # ---------------------------------------------------------
    if num_obs > 0:
        cols = 3; rows = (num_obs + cols - 1) // cols
        fig_detail = plt.figure(figsize=(18, 5 * rows))
        fig_detail.canvas.manager.set_window_title(f'2. Dettaglio Target {target_id}')
        fig_detail.suptitle(f"What does every drone in target {target_id} see?", fontsize=16)

        for i, obs_id in enumerate(observer_ids):
            ax = fig_detail.add_subplot(rows, cols, i + 1, projection='3d')
            data = df_target[df_target['observer_id'] == obs_id]
            
            if data.empty:
                ax.text2D(0.5, 0.5, "Nessun dato (Self)", transform=ax.transAxes, ha="center")
                continue

            ax.plot(data['claim_x'], data['claim_y'], data['claim_z'], color='red', linestyle='--', alpha=0.5, label='Claim')
            ax.plot(data['est_x'], data['est_y'], data['est_z'], color='blue', linewidth=2, label='Stima')
            ax.plot(data['true_x'], data['true_y'], data['true_z'], color='green', alpha=0.3, label='Vero')
            
            # Linee di errore visivo
            step = max(1, len(data) // 10)
            for idx in range(0, len(data), step):
                row = data.iloc[idx]
                ax.plot([row['est_x'], row['claim_x']], [row['est_y'], row['claim_y']], [row['est_z'], row['claim_z']], color='purple', alpha=0.3)

            last = data.iloc[-1]
            ax.set_title(f"D{obs_id} (Err: {last['discrepancy']:.2f}m)")
            if i==0: ax.legend(fontsize='x-small')

        plt.tight_layout()

    # NOTA: I grafici 3 (Comparativa) e 4 (Errori 2D) sono stati rimossi 
    # perché presenti nella nuova Dashboard unificata.

    print("Mostro i grafici... (Dovrebbero aprirsi 2 finestre)")
    plt.show()

if __name__ == "__main__":
    main()