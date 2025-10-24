#!/usr/bin/env python3
import matplotlib.pyplot as plt
import csv
import sys

def plot_csv(filename, samplerate):
    indices = []
    voltages = []

    # Legge il CSV
    with open(filename, 'r') as f:
        reader = csv.reader(f, delimiter=';')
        for row in reader:
            # Salta righe di commento o vuote
            if not row or row[0].startswith('#'):
                continue
            try:
                idx = int(row[0])
                vout = float(row[1])
                indices.append(idx)
                voltages.append(vout)
            except ValueError:
                continue  # ignora righe mal formattate

    if not indices:
        print("⚠️ Nessun dato trovato in", filename)
        return

    time = [i / samplerate for i in indices]
    x_label = f"Tempo [s] (Fs={samplerate:.0f} Hz)"
    x_data = time

    # Plot
    plt.figure(figsize=(10, 4))
    plt.plot(x_data, voltages, linewidth=1.0)
    plt.title(f"Uscita circuito: {filename}")
    plt.xlabel(x_label)
    plt.ylabel("Tensione [V]")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Uso:")
        print("  python plot_output.py <file.csv> <sample_rate>")
        print("\nEsempio:")
        print("  python plot_output.py output.csv 44100")
        sys.exit(1)

    filename = sys.argv[1]
    samplerate = float(sys.argv[2]) if len(sys.argv) > 2 else None
    plot_csv(filename, samplerate)
