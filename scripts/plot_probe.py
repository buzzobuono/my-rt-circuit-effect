import pandas as pd
import matplotlib.pyplot as plt

# Leggi il file CSV
df = pd.read_csv("probe.csv", sep=";")  # o sep=";" se il tuo file usa il punto e virgola
# Se il CSV ha intestazioni separate da spazi, puoi usare sep="\s+"

# Prendi il tempo
time = df.iloc[:, 0]

# Traccia tutte le altre colonne
plt.figure(figsize=(10,6))
for col in df.columns[1:]:
    plt.plot(time, df[col], label=col)

plt.xlabel("Time [s]")
plt.ylabel("Value")
plt.title("Probe Signals")
plt.legend()
plt.grid(True)
plt.show()
