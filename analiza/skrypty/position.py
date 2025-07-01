import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import find_peaks

# Wczytaj dane (dostosuj ścieżkę jeśli trzeba)
df = pd.read_csv('LOG00037.TXT', delim_whitespace=True, header=0)

# Konwersja kolumn na liczby
df['millis'] = pd.to_numeric(df['millis'], errors='coerce')
df['error'] = pd.to_numeric(df['error'], errors='coerce')
df = df.dropna(subset=['millis', 'error'])

# Normalizacja czasu
df['millis'] -= df['millis'].iloc[0]

df = df[df['millis'] <= 6000]

# Znajdź maksima (peaki) oscylacji błędu
peaks, _ = find_peaks(df['error'])

# Oblicz okres oscylacji (średni odstęp między kolejnymi peakami)
if len(peaks) > 1:
    peak_times = df['millis'].iloc[peaks].values
    periods = np.diff(peak_times)
    avg_period = np.mean(periods)
    print(f"Średni okres oscylacji: {avg_period:.1f} ms")
else:
    print("Nie znaleziono wystarczającej liczby maksimów do wyznaczenia okresu.")

# Wykres error vs czas z zaznaczonymi peakami
plt.figure(figsize=(10,5))
plt.plot(df['millis'], df['error'], label='error')
plt.xlabel('Czas [ms]')
plt.ylabel('Błąd (error)')
plt.title('Błąd PID w czasie')
plt.grid()
plt.legend()
plt.tight_layout()
plt.savefig('error_vs_time.png', dpi=300)
plt.show()