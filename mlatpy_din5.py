import pandas as pd
import matplotlib.pyplot as plt

FILE_RES, FILE_ANC = 'mlat_results_din.csv', 'anchors_trace.csv' 

res = pd.read_csv(FILE_RES)
anc = pd.read_csv(FILE_ANC)

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(projection='3d')

try:
    ax.plot(res['tx'], res['ty'], res['tz'], c='green', label='Target', linewidth=2)
    ax.scatter(res['ex'], res['ey'], res['ez'], c='blue', s=2, label='Stima')
except KeyError:
    ax.plot(res.iloc[:,0], res.iloc[:,1], res.iloc[:,2], c='green', label='Target')
    ax.scatter(res.iloc[:,3], res.iloc[:,4], res.iloc[:,5], c='blue', s=2, label='Stima')

num_droni = anc.shape[1] // 3

for i in range(num_droni):
    base_idx = i * 3
    x = anc.iloc[:, base_idx]
    y = anc.iloc[:, base_idx+1]
    z = anc.iloc[:, base_idx+2]
    
    ax.plot(x, y, z, alpha=0.3, label=f'Ancora {i+1}')
    ax.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], marker='^', s=50, c='red')

ax.set_title(f"Visualizzazione MLAT ({num_droni} Ancore)")
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
ax.legend()
plt.show()
