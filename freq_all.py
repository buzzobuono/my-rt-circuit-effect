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

# Calcola la FFT (Fast Fourier Transform)
N = len(channel_1)
fft_vals = np.fft.rfft(channel_1)
fft_freqs = np.fft.rfftfreq(N, d=1/sample_rate)

# Calcola il modulo (ampiezza) e converte in dB
magnitude = np.abs(fft_vals)
magnitude_db = 20 * np.log10(magnitude + 1e-10)  # eviti log(0)

# Trova la frequenza con ampiezza massima
peak_idx = np.argmax(magnitude)
peak_freq = fft_freqs[peak_idx]
peak_amp_db = magnitude_db[peak_idx]

# Crea il grafico
plt.figure(figsize=(12, 6))
plt.plot(fft_freqs, magnitude_db, linewidth=0.7)
plt.xlabel('Frequenza (Hz)')
plt.ylabel('Ampiezza (dB)')
plt.title(f'Spettro in frequenza - Canale 1 (Sample Rate: {sample_rate} Hz)')
plt.grid(True, alpha=0.3)
plt.xscale('log')

# Evidenzia il picco
plt.axvline(peak_freq, color='red', linestyle='--', alpha=0.7, label=f'Picco: {peak_freq:.1f} Hz')
plt.legend()

plt.tight_layout()
plt.show()

# Stampa info
duration = N / sample_rate
print(f"Durata: {duration:.2f} secondi")
print(f"Sample rate: {sample_rate} Hz")
print(f"Numero di campioni: {N}")
print(f"Frequenza di picco: {peak_freq:.2f} Hz ({peak_amp_db:.1f} dB)")
