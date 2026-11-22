import numpy as np
import matplotlib.pyplot as plt
from pyproj import Geod

# ---------------------------------------------------------
# 1. PARAMETRI WGS-84 & ORIGINE LTP
# ---------------------------------------------------------
a = 6378137.0             # Semiasse maggiore
b = 6356752.314245        # Semiasse minore (WGS-84 esatto)
f = (a - b) / a
e2 = 2*f - f**2

# Origine dell'LTP (es. latitude 45°, longitude 9°)
lat0_deg, lon0_deg, h0 = 45.0, 9.0, 0.0
lat0 = np.radians(lat0_deg)
lon0 = np.radians(lon0_deg)

# ---------------------------------------------------------
# 2. TRASFORMAZIONE ECEF <-> LTP (ENU)
# ---------------------------------------------------------
# ECEF dell'origine
N0 = a / np.sqrt(1 - e2 * np.sin(lat0)**2)
X0 = (N0 + h0) * np.cos(lat0) * np.cos(lon0)
Y0 = (N0 + h0) * np.cos(lat0) * np.sin(lon0)
Z0 = (N0 * (1 - e2) + h0) * np.sin(lat0)

# Matrice di rotazione da ENU (East, North, Up) a ECEF (dX, dY, dZ)
R_enu2ecef = np.array([
    [-np.sin(lon0), -np.sin(lat0)*np.cos(lon0),  np.cos(lat0)*np.cos(lon0)],
    [ np.cos(lon0), -np.sin(lat0)*np.sin(lon0),  np.cos(lat0)*np.sin(lon0)],
    [ 0,             np.cos(lat0),               np.sin(lat0)]
])

def enu_to_geodetic(e, n, u):
    """Converte offset locali ENU in lat, lon, alt geodetiche."""
    # 1. ENU -> ECEF
    enu = np.array([e, n, u])
    d_ecef = R_enu2ecef @ enu
    X = X0 + d_ecef[0]
    Y = Y0 + d_ecef[1]
    Z = Z0 + d_ecef[2]
    
    # 2. ECEF -> Geodetiche (Algoritmo iterativo di Bowring)
    p = np.sqrt(X**2 + Y**2)
    theta = np.arctan2(Z * a, p * b)
    
    lat = np.arctan2(
        Z + (e2 * a**3 / b) * np.sin(theta)**3,
        p - (e2 * a) * np.cos(theta)**3
    )
    lon = np.arctan2(Y, X)
    
    N = a / np.sqrt(1 - e2 * np.sin(lat)**2)
    alt = p / np.cos(lat) - N
    
    return np.degrees(lat), np.degrees(lon), alt

# ---------------------------------------------------------
# 3. GENERAZIONE CERCHIO LTP DI RAGGIO 1 km (1000m)
# ---------------------------------------------------------
R_target = 1000.0  # 1 km in metri
azimuths = np.linspace(0, 360, 361)  # Angoli da 0 a 360 gradi

# Inizializziamo il solutore geodesico WGS-84
geod = Geod(ellps='WGS84')

errori_m = []

for az in azimuths:
    az_rad = np.radians(az)
    
    # Coordinate nel piano tangente locale (East, North, Up)
    e_point = R_target * np.sin(az_rad)
    n_point = R_target * np.cos(az_rad)
    u_point = 0.0
    
    # Distanza sul piano LTP (euclidea 2D)
    dist_ltp = np.sqrt(e_point**2 + n_point**2)  # Sempre 1000.0 m
    
    # Conversione in coordinate geodetiche
    lat_i, lon_i, _ = enu_to_geodetic(e_point, n_point, u_point)
    
    # Calcolo della vera distanza geodetica sull'ellissoide
    _, _, dist_geodetica = geod.inv(lon0_deg, lat0_deg, lon_i, lat_i)
    
    # Errore = Distanza_LTP - Distanza_Geodetica
    errore = 100*(dist_ltp - dist_geodetica)/dist_ltp  # Normalizzato
    print(dist_ltp, dist_geodetica, errore)
    errori_m.append(errore)


# ---------------------------------------------------------
# 4. GRAFICO DELL'ERRORE AL VARIARE DELL'AZIMUTH
# ---------------------------------------------------------
plt.figure(figsize=(10, 5))
plt.plot(azimuths, np.array(errori_m) * 1000, color='b', lw=2)  # errore in mm
plt.title(f"Error between geodetic and LTP distances (Lat {lat0_deg}°)")
plt.xlabel("Azimuth North [deg]")
plt.ylabel("Error [%]")
plt.grid(True, alpha=0.3)
plt.show()

print(f"Errore massimo su 1 km: {np.max(np.abs(errori_m))*1000:.3f} mm")