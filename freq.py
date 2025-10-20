import numpy as np
import matplotlib.pyplot as plt
from scipy.io import wavfile
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("wav_file", help="Il file WAV da analizzare")
args = parser.parse_args()

# Leggi il file WAV
file_path = args.wav_file
sample_rate, data = wavfile.read(file_path)

# Estrai solo il primo canale (in caso di stereo)
if len(data.shape) > 1:
    channel_1 = data[:, 0]
else:
    channel_1 = data

# Crea lo spettrogramma
plt.figure(figsize=(12, 6))
plt.specgram(channel_1, Fs=sample_rate, NFFT=2048, noverlap=1024, cmap='inferno')

plt.xlabel('Tempo (s)')
plt.ylabel('Frequenza (Hz)')
plt.title(f'Spettrogramma - Canale 1 (Sample Rate: {sample_rate} Hz)')
plt.colorbar(label='Intensità (dB)')
plt.tight_layout()
plt.show()

# Stampa info
duration = len(channel_1) / sample_rate
print(f"Durata: {duration:.2f} secondi")
print(f"Sample rate: {sample_rate} Hz")
print(f"Numero di campioni: {len(channel_1)}")
