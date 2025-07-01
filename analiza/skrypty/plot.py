import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# --- PARAMETERS ---
mm_per_count = 1  # <-- UPDATE THIS with your encoder's mm per count
wheel_base = 150  # mm

# --- LOAD DATA ---
# Adjust delimiter if needed (',' for CSV, '\t' for tab, etc.)
df = pd.read_csv('../autotune/przejazd-autotune-nieoptymalny.TXT', delim_whitespace=True, header=0, usecols=[0,1,2])
df.columns = ['millis', 'enc_left', 'enc_right']

# --- KONWERSJA NA LICZBY ---
df['millis'] = pd.to_numeric(df['millis'], errors='coerce')
df['enc_left'] = pd.to_numeric(df['enc_left'], errors='coerce')
df['enc_right'] = pd.to_numeric(df['enc_right'], errors='coerce')
df = df.dropna(subset=['millis', 'enc_left', 'enc_right'])

# --- NORMALIZE TIME ---
df['millis'] -= df['millis'].iloc[0]

# --- CALCULATE INCREMENTAL DISTANCES ---
df['d_enc_left'] = df['enc_left'].diff().fillna(0)
df['d_enc_right'] = df['enc_right'].diff().fillna(0)
df['d_left_mm'] = df['d_enc_left'] * mm_per_count
df['d_right_mm'] = df['d_enc_right'] * mm_per_count

# --- DEAD RECKONING TRACK ESTIMATION ---
theta = 0
x, y = [0], [0]
for i in range(1, len(df)):
    dl = df['d_left_mm'].iloc[i]
    dr = df['d_right_mm'].iloc[i]
    dc = (dl + dr) / 2
    dtheta = (dr - dl) / wheel_base
    theta += dtheta
    x.append(x[-1] + dc * np.cos(theta))
    y.append(y[-1] + dc * np.sin(theta))

# --- FIRST ROTATION ---
angle1_deg = -3
angle1_rad = np.deg2rad(angle1_deg)
rotate_after1 = 0

x_rot1 = np.array(x).copy()
y_rot1 = np.array(y).copy()
x0_1, y0_1 = x[rotate_after1], y[rotate_after1]
for i in range(rotate_after1, len(x)):
    dx = x_rot1[i] - x0_1
    dy = y_rot1[i] - y0_1
    x_rot1[i] = x0_1 + dx * np.cos(angle1_rad) - dy * np.sin(angle1_rad)
    y_rot1[i] = y0_1 + dx * np.sin(angle1_rad) + dy * np.cos(angle1_rad)

# --- SECOND ROTATION (on already rotated data) ---
angle2_deg = -10
angle2_rad = np.deg2rad(angle2_deg)
rotate_after2 = 400

x_rot2 = x_rot1.copy()
y_rot2 = y_rot1.copy()
x0_2, y0_2 = x_rot1[rotate_after2], y_rot1[rotate_after2]
for i in range(rotate_after2, len(x_rot2)):
    dx = x_rot2[i] - x0_2
    dy = y_rot2[i] - y0_2
    x_rot2[i] = x0_2 + dx * np.cos(angle2_rad) - dy * np.sin(angle2_rad)
    y_rot2[i] = y0_2 + dx * np.sin(angle2_rad) + dy * np.cos(angle2_rad)

# --- PLOTTING ---
img = mpimg.imread('trasa.png')
offset = 150
xmin, xmax = min(x_rot2) - offset, max(x_rot2) + offset
ymin, ymax = min(y_rot2) - offset, max(y_rot2) + offset

plt.figure(figsize=(8,6))
plt.imshow(img, extent=[xmin, xmax, ymin, ymax], aspect='auto')
plt.plot(x_rot2, y_rot2, label='Trasa robota', color='blue')
plt.xlabel('X (mm)')
plt.ylabel('Y (mm)')
plt.title('Trasa robota na podstawie danych enkoderów')
plt.axis('equal')
plt.xlim(xmin, xmax)
plt.ylim(ymin, ymax)
plt.legend()
plt.savefig('wykres_trasy.png', dpi=300, bbox_inches='tight')