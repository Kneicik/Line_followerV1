import numpy as np

# Wczytaj dane, automatycznie wykryj separator
# data = np.genfromtxt('../heurystyczna/ziegler-nichols/przejazd-ziegler.TXT', names=True, dtype=None, encoding=None)
data = np.genfromtxt('../autotune/przejazd-autotune-optymalny.TXT', names=True, dtype=None, encoding=None)

# Suma kolumny "lost"
total_lost = np.sum(data['lost'])

# Średnia wartość bezwzględna "error" tam gdzie "lost" == 0
error_when_not_lost = np.abs(data['error'][data['lost'] == 0])
avg_error_not_lost = np.mean(error_when_not_lost) if len(error_when_not_lost) > 0 else float('nan')

# Wariancja próby (domyślnie ddof=1)
var_error_not_lost = np.var(error_when_not_lost, ddof=1) if len(error_when_not_lost) > 1 else float('nan')

print(f"Total lost: {total_lost}")
print(f"Average absolute error where lost == 0: {avg_error_not_lost:.2f}")
print(f"Sample variance of absolute error where lost == 0: {var_error_not_lost:.2f}")