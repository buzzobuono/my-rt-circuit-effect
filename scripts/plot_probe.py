import argparse
import pandas as pd
import matplotlib.pyplot as plt

# --- Parser degli argomenti ---
parser = argparse.ArgumentParser(description="Plot dei segnali da un file CSV di probe.")
parser.add_argument("file", help="Nome del file CSV da leggere (es: probe.csv)")
parser.add_argument("--sep", default=";", help="Separatore di campo (default: ';')")
parser.add_argument("--title", default="Probe Signals", help="Titolo del grafico")
parser.add_argument("--save", help="Salva il grafico su file (es: output.png invece di mostrarlo)")

args = parser.parse_args()

# --- Lettura CSV ---
df = pd.read_csv(args.file, sep=args.sep)
time = df.iloc[:, 0]

# --- Plot ---
plt.figure(figsize=(10,6))
for col in df.columns[1:]:
    plt.plot(time, df[col], label=col)

plt.xlabel("Time [s]")
plt.ylabel("Value")
plt.title(f"{args.title} - {args.file}")
plt.legend()
plt.grid(True)

plt.show()