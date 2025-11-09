#!/usr/bin/env python3
"""
FFT Phase Analyzer - Confronta sfasamento tra due file WAV
Uso: python fft_phase.py input.wav output.wav
"""

import sys
import numpy as np
from scipy.io import wavfile
from scipy.fft import fft, fftfreq

def analyze_phase(input_file, output_file):
    # Carica file WAV
    try:
        fs_in, input_wav = wavfile.read(input_file)
        fs_out, output_wav = wavfile.read(output_file)
    except Exception as e:
        print(f"ERRORE: Impossibile leggere i file WAV: {e}", file=sys.stderr)
        sys.exit(1)
    
    # Verifica sample rate
    if fs_in != fs_out:
        print(f"ERRORE: Sample rate diversi! Input={fs_in}Hz, Output={fs_out}Hz", file=sys.stderr)
        sys.exit(1)
    
    fs = fs_in
    
    # Converti in float e normalizza
    input_wav = input_wav.astype(float)
    output_wav = output_wav.astype(float)
    
    input_wav = input_wav / np.max(np.abs(input_wav))
    output_wav = output_wav / np.max(np.abs(output_wav))
    
    # Assicura stessa lunghezza
    min_len = min(len(input_wav), len(output_wav))
    input_wav = input_wav[:min_len]
    output_wav = output_wav[:min_len]
    
    # FFT
    N = len(input_wav)
    input_fft = fft(input_wav)
    output_fft = fft(output_wav)
    freqs = fftfreq(N, 1/fs)
    
    # Calcola fase (unwrap per continuità)
    phase_in = np.angle(input_fft)
    phase_out = np.angle(output_fft)
    phase_diff = np.unwrap(phase_out - phase_in)
    phase_diff_deg = np.degrees(phase_diff)
    
    # Calcola attenuazione (magnitude)
    mag_in = np.abs(input_fft)
    mag_out = np.abs(output_fft)
    
    # Evita divisione per zero
    attenuation_db = np.zeros_like(mag_in)
    mask = mag_in > 1e-10
    attenuation_db[mask] = 20 * np.log10(mag_out[mask] / mag_in[mask])
    
    # Output: solo frequenze positive
    positive_mask = freqs > 0
    freqs_pos = freqs[positive_mask]
    phase_pos = phase_diff_deg[positive_mask]
    atten_pos = attenuation_db[positive_mask]
    
    # Trova frequenze di test (picchi nello spettro di input)
    mag_pos = mag_in[positive_mask]
    peak_threshold = 0.1 * np.max(mag_pos)
    
    # Header
    print("=" * 80)
    print(f"FFT PHASE ANALYSIS")
    print("=" * 80)
    print(f"Input file:   {input_file}")
    print(f"Output file:  {output_file}")
    print(f"Sample rate:  {fs} Hz")
    print(f"Samples:      {N}")
    print(f"Duration:     {N/fs:.3f} s")
    print("=" * 80)
    print()
    
    # Frequenze standard da testare (audio range)
    test_freqs = [40, 80, 110, 160, 220, 330, 440, 660, 880, 1000, 1500, 2000, 3000, 5000, 10000]
    
    print(f"{'Freq [Hz]':>10} | {'Phase [deg]':>12} | {'Atten [dB]':>12}")
    print("-" * 40)
    
    for f_test in test_freqs:
        if f_test > fs/2:
            continue  # Skip frequenze oltre Nyquist
        
        # Trova indice più vicino
        idx = np.argmin(np.abs(freqs_pos - f_test))
        f_actual = freqs_pos[idx]
        
        # Salta se troppo lontano dalla frequenza target
        if abs(f_actual - f_test) > 10:
            continue
        
        phase_val = phase_pos[idx]
        atten_val = atten_pos[idx]
        
        print(f"{f_test:10.1f} | {phase_val:12.2f} | {atten_val:12.2f}")
    
    print()
    print("=" * 80)
    
    # Statistiche globali
    # Considera solo banda audio rilevante (20Hz - 20kHz)
    audio_mask = (freqs_pos >= 20) & (freqs_pos <= 20000)
    
    if np.any(audio_mask):
        phase_mean = np.mean(phase_pos[audio_mask])
        phase_std = np.std(phase_pos[audio_mask])
        atten_mean = np.mean(atten_pos[audio_mask])
        atten_std = np.std(atten_pos[audio_mask])
        
        print("STATISTICHE BANDA AUDIO (20Hz - 20kHz):")
        print(f"  Phase medio:     {phase_mean:8.2f}° ± {phase_std:.2f}°")
        print(f"  Attenuazione:    {atten_mean:8.2f} dB ± {atten_std:.2f} dB")
        print("=" * 80)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Uso: python fft_phase.py <input.wav> <output.wav>", file=sys.stderr)
        print()
        print("Esempio:")
        print("  python fft_phase.py bass_input.wav bass_filtered.wav")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    
    analyze_phase(input_file, output_file)