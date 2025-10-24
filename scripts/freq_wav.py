import numpy as np
import matplotlib.pyplot as plt
from scipy.io import wavfile
import argparse

parser = argparse.ArgumentParser(description="Analizza lo spettro di uno o due file WAV (prima/dopo il filtro).")
parser.add_argument("wav_file", help="Il file WAV da analizzare (es. output filtrato)")
parser.add_argument("--compare", help="File WAV originale per confronto", default=None)
parser.add_argument("--fc", type=float, help="Frequenza di taglio prevista (Hz)", default=None)
args = parser.parse_args()

def load_wav_channel(path):
    sr, data = wavfile.read(path)
    if data.ndim > 1:
        data = data[:, 0]  # solo primo canale
    data = data.astype(float)
    return sr, data

def compute_fft(data, sr):
    N = len(data)
    fft_vals = np.fft.rfft(data)
    fft_freqs = np.fft.rfftfreq(N, 1/sr)
    magnitude = np.abs(fft_vals)
    magnitude_db = 20 * np.log10(magnitude + 1e-10)
    return fft_freqs, magnitude_db

# --- Caricamento ---
sr1, data1 = load_wav_channel(args.wav_file)
freqs1, mag1 = compute_fft(data1, sr1)

plt.figure(figsize=(12, 6))
plt.plot(freqs1, mag1, label=f"{args.wav_file}", linewidth=0.8)

if args.compare:
    sr2, data2 = load_wav_channel(args.compare)
    if sr1 != sr2:
        raise ValueError("I file devono avere lo stesso sample rate.")
    freqs2, mag2 = compute_fft(data2, sr2)
    plt.plot(freqs2, mag2, label=f"{args.compare}", linewidth=0.8, alpha=0.7)

# --- Visualizzazione ---
plt.xscale('log')
plt.xlabel("Frequenza (Hz)")
plt.ylabel("Ampiezza (dB)")
plt.title("Analisi spettro audio (prima e dopo filtro)")
plt.grid(alpha=0.3)
plt.legend()

# Mostra linea della frequenza di taglio
if args.fc:
    plt.axvline(args.fc, color='red', linestyle='--', label=f"Taglio previsto {args.fc:.0f} Hz")

plt.tight_layout()
plt.show()

# --- Info principali ---
duration = len(data1) / sr1
peak_idx = np.argmax(mag1)
print(f"Durata: {duration:.2f} s")
print(f"Sample rate: {sr1} Hz")
print(f"Frequenza di picco (output): {freqs1[peak_idx]:.2f} Hz")
