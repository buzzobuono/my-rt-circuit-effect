import numpy as np
import matplotlib.pyplot as plt
import argparse
import csv

parser = argparse.ArgumentParser(description="Analizza lo spettro di uno o due file CSV (prima/dopo il filtro).")
parser.add_argument("csv_file", help="File CSV da analizzare (es. output filtrato)")
parser.add_argument("--compare", help="File CSV originale per confronto", default=None)
parser.add_argument("--sr", type=float, required=True, help="Sample rate in Hz")
parser.add_argument("--fc", type=float, help="Frequenza di taglio prevista (Hz)", default=None)
args = parser.parse_args()


def load_csv_voltage(path):
    """Legge un file CSV con colonne 'index;voltage_out'."""
    indices = []
    voltages = []
    with open(path, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=';')
        header = next(reader, None)  # salta intestazione se presente
        for row in reader:
            if len(row) < 2:
                continue
            try:
                indices.append(int(row[0]))
                voltages.append(float(row[1]))
            except ValueError:
                # salta righe non valide
                continue
    data = np.array(voltages, dtype=float)
    return data


def compute_fft(data, sr):
    N = len(data)
    fft_vals = np.fft.rfft(data)
    fft_freqs = np.fft.rfftfreq(N, 1/sr)
    magnitude = np.abs(fft_vals)
    magnitude_db = 20 * np.log10(magnitude + 1e-10)
    return fft_freqs, magnitude_db


# --- Caricamento ---
data1 = load_csv_voltage(args.csv_file)
freqs1, mag1 = compute_fft(data1, args.sr)

plt.figure(figsize=(12, 6))
plt.plot(freqs1, mag1, label=f"{args.csv_file}", linewidth=0.8)

if args.compare:
    data2 = load_csv_voltage(args.compare)
    if len(data1) != len(data2):
        print("⚠️ Attenzione: i file hanno lunghezze diverse, il confronto potrebbe non essere preciso.")
    freqs2, mag2 = compute_fft(data2, args.sr)
    plt.plot(freqs2, mag2, label=f"{args.compare}", linewidth=0.8, alpha=0.7)

# --- Visualizzazione ---
plt.xscale('log')
plt.xlabel("Frequenza (Hz)")
plt.ylabel("Ampiezza (dB)")
plt.title("Analisi spettro segnale (prima e dopo filtro)")
plt.grid(alpha=0.3)
plt.legend()

# Mostra linea della frequenza di taglio
if args.fc:
    plt.axvline(args.fc, color='red', linestyle='--', label=f"Taglio previsto {args.fc:.0f} Hz")

plt.tight_layout()
plt.show()

# --- Info principali ---
duration = len(data1) / args.sr
peak_idx = np.argmax(mag1)
print(f"Durata: {duration:.2f} s")
print(f"Sample rate: {args.sr} Hz")
print(f"Frequenza di picco (output): {freqs1[peak_idx]:.2f} Hz")
