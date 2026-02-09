import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def run_swarmraft_analysis(csv_file='tdma_security_log.csv'):
    # 1. Caricamento dati aggiornati
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Errore: {csv_file} non trovato.")
        return

    # Parametri Consenso
    N = df['observer_id'].nunique() + 1
    f = (N - 1) // 2
    threshold_consensus = N - f

    # 2. Trasformazione in voti
    df['vote'] = df['alarm'].apply(lambda x: -1 if x else 1)
    
    # Calcolo Errore GPS vs Errore Recuperato (SwarmRaft)
    # Errore GPS = distanza tra GPS dichiarato e Verità
    df['gps_error'] = np.sqrt((df['claim_x'] - df['true_x'])**2 + 
                              (df['claim_y'] - df['true_y'])**2 + 
                              (df['claim_z'] - df['true_z'])**2)
    
    # Errore Recuperato = distanza tra Posizione di Consenso e Verità
    df['rec_error'] = np.sqrt((df['rec_x'] - df['true_x'])**2 + 
                              (df['rec_y'] - df['true_y'])**2 + 
                              (df['rec_z'] - df['true_z'])**2)

    # Aggregazione per Target 0 (l'attaccante)
    target_0 = df[df['sender_id'] == 0].groupby('time').first().reset_index()
    voting_res = df[df['sender_id'] == 0].groupby('time')['vote'].sum().reset_index()

    # --- VISUALIZZAZIONE PROFESSIONALE ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

    # PLOT 1: CONSENSO (Votazione)
    ax1.step(voting_res['time'], voting_res['vote'], color='#1f77b4', label='Total Consensus Votes ($S_i$)')
    ax1.axhline(y=-(threshold_consensus-1), color='red', linestyle='--', alpha=0.6, label='Malevolent Threshold')
    ax1.axhline(y=(threshold_consensus-1), color='green', linestyle='--', alpha=0.6, label='Honest Threshold')
    ax1.set_ylabel("Sum of Votes")
    ax1.set_title(f"SwarmRaft Consensus Analysis")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='lower left')

    # PLOT 2: RECOVERY (Confronto Errori)
    ax2.plot(target_0['time'], target_0['gps_error'], color='red', alpha=0.5, label='GPS Error (Under Attack)')
    ax2.plot(target_0['time'], target_0['rec_error'], color='green', linewidth=2, label='Error Recovered (SwarmRaft)')
    
    # Area di attacco
    ax2.axvspan(200, 300, color='red', alpha=0.05, label='GPS Spoofing Attack Active')

    ax2.set_ylabel("Position Error [m]")
    ax2.set_xlabel("Simulation Time [s]")
    ax2.set_title("Recovery Effectiveness: Corrupted GPS vs. Swarm Consensus")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper left')

    plt.tight_layout()
    plt.savefig('validazione_swarmraft_finale.png')
    print("Analisi completata. Grafico salvato come 'validazione_swarmraft_finale.png'")
    plt.show()

if __name__ == "__main__":
    run_swarmraft_analysis()
