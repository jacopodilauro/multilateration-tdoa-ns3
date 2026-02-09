import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys

def main():
    filename = 'tdma_security_log.csv'
    try:
        df = pd.read_csv(filename)
    except FileNotFoundError:
        print(f"ERRORE: File '{filename}' non trovato.")
        return

    target_id = 0 
    if len(sys.argv) > 1:
        try: target_id = int(sys.argv[1])
        except ValueError: pass

    df_target = df[df['sender_id'] == target_id]
    if df_target.empty:
        print("Dati non trovati.")
        return

    # 1. Filtriamo esattamente i 5 osservatori
    observer_ids = sorted(df_target['observer_id'].unique())
    valid_observers = [o for o in observer_ids if not df_target[df_target['observer_id'] == o].empty]
    
    # Se per qualche motivo sono più di 5, limitiamo ai primi 5 per mantenere il layout 2+3 richiesto
    valid_observers = valid_observers[:5]

    # 2. Calcolo limiti assi comuni per coerenza scientifica
    all_x = pd.concat([df_target['true_x'], df_target['claim_x'], df_target['est_x']])
    all_y = pd.concat([df_target['true_y'], df_target['claim_y'], df_target['est_y']])
    all_z = pd.concat([df_target['true_z'], df_target['claim_z'], df_target['est_z']])
    
    x_lim = (all_x.min(), all_x.max())
    y_lim = (all_y.min(), all_y.max())
    z_lim = (all_z.min(), all_z.max())

    # 3. Creazione Figura con Layout 2 sopra / 3 sotto
    # Aumentiamo la dimensione complessiva per rendere i grafici "più grandi"
    fig = plt.figure(figsize=(20, 12))
    
    # Creiamo una griglia di base di 2 righe e 6 colonne (Minimo Comune Multiplo di 2 e 3)
    gs = GridSpec(2, 6, figure=fig)
    plt.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.90, wspace=0.4, hspace=0.3)
    
    fig.suptitle(f"Analisi di Sicurezza Sciame: Visuale degli Osservatori sul Target {target_id}", 
                 fontsize=20, fontweight='bold', y=0.96)

    # Definiamo le posizioni nella griglia 2x6
    # Riga 0: 2 grafici che occupano 3 colonne ciascuno
    # Riga 1: 3 grafici che occupano 2 colonne ciascuno
    positions = [
        gs[0, 0:3], gs[0, 3:6], # Top row
        gs[1, 0:2], gs[1, 2:4], gs[1, 4:6] # Bottom row
    ]

    for i, obs_id in enumerate(valid_observers):
        ax = fig.add_subplot(positions[i], projection='3d')
        data = df_target[df_target['observer_id'] == obs_id]
        
        # Plot Traiettorie
        ax.plot(data['true_x'], data['true_y'], data['true_z'], color='green', alpha=0.3, linewidth=1.5, label='Ground Truth')
        ax.plot(data['claim_x'], data['claim_y'], data['claim_z'], color='red', linestyle='--', alpha=0.5, label='GPS Claim')
        ax.plot(data['est_x'], data['est_y'], data['est_z'], color='blue', linewidth=2.5, label=f'Stima Drone {obs_id}')
        
        # Titolo e Assi
        ax.set_title(f"PROSPETTIVA DRONE {obs_id}", fontsize=14, fontweight='semibold', pad=10)
        ax.set_xlim(x_lim); ax.set_ylim(y_lim); ax.set_zlim(z_lim)
        
        # Etichette (opzionali se vuoi pulire ulteriormente il grafico)
        ax.set_xlabel('X [m]', fontsize=10)
        ax.set_ylabel('Y [m]', fontsize=10)
        ax.set_zlabel('Z [m]', fontsize=10)
        
        # Angolazione per una vista chiara della traiettoria
        ax.view_init(elev=20, azim=-35)
        
        if i == 0:
            ax.legend(loc='upper left', fontsize='medium')

    print(f"Layout 2+3 generato con successo per {len(valid_observers)} droni.")
    plt.show()

if __name__ == "__main__":
    main()
