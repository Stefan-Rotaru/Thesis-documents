import pandas as pd
import matplotlib.pyplot as plt
import statistics
from math import log10, floor
import numpy as np

# 1) Load the CSV (adjust the filename/path as needed)
df = pd.read_csv('2sin.csv')

# 2) Plot windspeed_z vs time
plt.figure(figsize=(10, 5))
time =  df['time']  # Adjust time to start at 0s
dt = time[1]-time[0]
windspeed_z = df['windspeed_z']
time2 = [ti - 80.8 for ti in time]

# time2 = np.zeros(len(time))
# for i in range (1, len(time2)):
#     time2[i] = time2[i-1] + dt
meane =statistics.mean(windspeed_z)
print(f"Mean windspeed_z: {meane}")
plt.plot(time2, windspeed_z)

def round_to_sig_figs(num, sig_figs=3):
    if num == 0:
        return 0
    else:
        return round(num, sig_figs - int(floor(log10(abs(num)))) - 1)

u_fluct = (windspeed_z - (meane))


print(statistics.mean(np.abs(u_fluct))/meane * 100)

Ubar   = np.mean(windspeed_z)        # don't round here
u_fluct = windspeed_z - Ubar
urms   = np.sqrt(np.mean(u_fluct**2))  # same as np.std(windspeed_z, ddof=0)
TI     = 100 * urms / Ubar

print(TI)

# 3) Beautify
plt.xlabel('Time [s]')
plt.ylabel('Windspeed [m/s]')
plt.title('Windspeed over time')
plt.axhline(statistics.mean(windspeed_z), color='black', linestyle='--', linewidth=1, label=f'Mean Windspeed U={round_to_sig_figs(meane)}m/s')
plt.grid(True)
plt.legend()
plt.xlim(left=0)  # Start x-axis at 0
plt.xlim(right=40)  # End x-axis at the maximum time value
plt.ylim(bottom = 4.5, top = 7.5)
plt.tight_layout()

# 4) Show
plt.show()