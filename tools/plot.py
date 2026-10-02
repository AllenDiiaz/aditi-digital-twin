import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

# Re-plot a CSV recorded by velocity_profiler.py.
# Usage: python3 plot.py <csv_path>
if len(sys.argv) < 2:
    print("Usage: python3 plot.py <csv_path>")
    sys.exit(1)

csv_filename = sys.argv[1]

if not os.path.exists(csv_filename):
    print(f"File not found: {csv_filename}")
    exit()

print(f"Reading data: {csv_filename} ...")
df = pd.read_csv(csv_filename)

# Split commanded and measured samples.
df_sim = df[df['type'] == 'sim_cmd']
df_real = df[df['type'] == 'real_act']

# Plot absolute velocity (speed).
plt.figure(figsize=(12, 6))

# Sim command: blue dashed line.
if not df_sim.empty:
    plt.plot(df_sim['time'].to_numpy(), np.abs(df_sim['vel_x'].to_numpy()),
             'b--', label='Sim Command (Speed)', linewidth=2)

# Real robot: red solid line.
if not df_real.empty:
    plt.plot(df_real['time'].to_numpy(), np.abs(df_real['vel_x'].to_numpy()),
             'r-', label='Real Robot (Speed)', linewidth=2, alpha=0.8)

plt.title(f"Sim-to-Real Speed Tracking (Re-plot from CSV)", fontsize=14)
plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.legend()
plt.grid(True, which='both', linestyle='--', alpha=0.7)

# Save next to the input file.
output_filename = os.path.splitext(csv_filename)[0] + "_replot.png"
plt.savefig(output_filename)

print(f"Plot saved: {output_filename}")
print(f"Location: {os.path.abspath(output_filename)}")
