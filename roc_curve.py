import pandas as pd
import matplotlib.pyplot as plt
from sklearn.metrics import roc_curve, auc
import numpy as np
import sys

# Stile grafico
plt.rcParams.update({'font.size': 12, 'figure.autolayout': True})

def plot_gradient_roc(filename='tdma_security_log.csv'):
    print(f"--- Generazione Gradient ROC per {filename} ---")
    
    try:
        df = pd.read_csv(filename)
    except FileNotFoundError:
        print(f"ERRORE: File '{filename}' non trovato.")
        return

    # 1. GROUND TRUTH (Verità)
    # Filtriamo solo il drone 0 (Target)
    df_target = df[df['sender_id'] == 0].copy()
    if df_target.empty:
        print("Nessun dato per Drone 0.")
        return

    # L'attacco è vero se siamo oltre i 200s
    df_target['label_true'] = (df_target['time'] >= 200.0).astype(int)

    y_true = df_target['label_true']
    y_scores = df_target['discrepancy']

    # 2. CALCOLO ROC
    fpr, tpr, thresholds = roc_curve(y_true, y_scores)
    roc_auc = auc(fpr, tpr)

    # 3. PLOT AVANZATO (Gradient)
    plt.figure(figsize=(10, 8))
    
    # Disegniamo la linea di base grigia sottile per continuità
    plt.plot(fpr, tpr, color='gray', alpha=0.3, linewidth=1)

    # Disegniamo i PUNTI colorati in base alla soglia
    # Limitiamo il colore massimo a 20m per non "schiacciare" la scala colori 
    # (perché l'EKF all'inizio può avere errori enormi che rovinerebbero la visualizzazione)
    sc = plt.scatter(fpr, tpr, c=thresholds, cmap='turbo', s=15, 
                     vmin=0, vmax=20, label='Soglia Variabile (0-20m)')
    
    # Aggiungiamo la barra laterale che spiega i colori
    cbar = plt.colorbar(sc)
    cbar.set_label('Valore della Soglia [metri]', rotation=270, labelpad=15)

    # --- EVIDENZIAMO PUNTI CHIAVE ---
    # Cerchiamo le soglie specifiche che ti interessano
    points_of_interest = [4.0, 10.0, 15.0]
    colors = ['green', 'red', 'purple']
    
    for val, col in zip(points_of_interest, colors):
        idx = (np.abs(thresholds - val)).argmin()
        plt.scatter(fpr[idx], tpr[idx], color=col, s=150, edgecolors='black', zorder=10,
                    label=f'Soglia {val}m (TPR={tpr[idx]:.2f})')

    # Linea casuale
    plt.plot([0, 1], [0, 1], color='navy', linestyle='--', alpha=0.6)

    plt.xlim([-0.02, 1.02])
    plt.ylim([-0.02, 1.05])
    plt.xlabel('False Positive Rate (Falsi Allarmi)')
    plt.ylabel('True Positive Rate (Attacchi Rilevati)')
    plt.title(f'ROC Curve con Mappatura Soglie (AUC = {roc_auc:.3f})')
    plt.legend(loc="lower right")
    plt.grid(True, linestyle='--', alpha=0.5)

    print(f"Grafico generato. AUC: {roc_auc:.4f}")
    plt.show()

if __name__ == "__main__":
    plot_gradient_roc()
