import numpy as np
import matplotlib.pyplot as plt
from scipy.io import wavfile
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("wav_file", help="The file")
args = parser.parse_args()

# Leggi il file WAV
file_path = args.wav_file
sample_rate, data = wavfile.read(file_path)

# Estrai solo il primo canale
if len(data.shape) > 1:
    # File stereo o multicanale
    channel_1 = data[:, 0]
else:
    # File mono
    channel_1 = data

# Crea l'asse temporale in secondi
time = np.arange(len(channel_1)) / sample_rate

# Crea il grafico
plt.figure(figsize=(12, 6))
plt.plot(time, channel_1, linewidth=0.5)
plt.xlabel('Tempo (s)')
plt.ylabel('Ampiezza')
plt.title(f'Forma d\'onda - Canale 1 (Sample Rate: {sample_rate} Hz)')
plt.grid(True, alpha=0.3)

# Abilita lo zoom interattivo
plt.tight_layout()
plt.show()

print(f"Durata: {time[-1]:.2f} secondi")
print(f"Sample Rate: {sample_rate} Hz")
print(f"Numero di campioni: {len(channel_1)}")