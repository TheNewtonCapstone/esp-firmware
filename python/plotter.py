import pandas as pd
import matplotlib.pyplot as plt

# Define target value
TARGET_VELOCITY = 20
VELOCITY_RANGE =  478
NORMALIZED_TARGET_VELOCITY = TARGET_VELOCITY / 100
file_path = 'log/2021-09-26-20-10-10'
save_path = file_path + '.png'
file_path = file_path + '.csv'

# Read the CSV file
df = pd.read_csv(file_path)
print(df.head())    

# Set up the plotting style
# plt.style.use('seaborn')
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 12))
fig.tight_layout(pad=3.0)

# Plot 1: Velocity vs Time with target line
ax1.plot(df['timestamp'], df['velocity'], linewidth=2, color='#2E86C1', label='Velocity')
# ax1.plot(df['current_time'], df['output'], linewidth=2, color='#27AE60', label='Output')
ax1.axhline(y=TARGET_VELOCITY, color='#E74C3C', linestyle='--', label='Target Velocity')
ax1.set_xlabel('Time')
ax1.set_ylabel('Velocity')
ax1.set_title('Velocity vs Time')
ax1.grid(True)
ax1.legend()

# Plot 2: Error vs Time
ax2.plot(df['timestamp'], df['position'], linewidth=2, color='#E74C3C')
ax2.axhline(y=0, color='#2E86C1', linestyle='--', label='Position')
ax2.set_xlabel('Time')
ax2.set_ylabel('Position')
ax2.set_title('Position vs Time')
ax2.grid(True)
ax2.legend()

# Plot 3: Output vs Error
# ax3.plot(df['current_time'], df['normalized_error'], alpha=0.6, color='#27AE60', label='Normalized Error')
# ax3.plot(df['current_time'], df['output'], alpha=0.6, color='#E74C3C', label='Output')
# velocity = df['current_velocity'] / VELOCITY_RANGE
# ax3.plot(df['current_time'], velocity, alpha=0.6, color='#2E86C1', label='Velocity')
# NORMALIZED_TARGET_VELOCITY = TARGET_VELOCITY / VELOCITY_RANGE
# ax3.axhline(y=NORMALIZED_TARGET_VELOCITY, color='#E74C3C', linestyle='--', label='Target Velocity')
# ax3.set_xlabel('Time')
# ax3.set_ylabel('Output')

# ax3.grid(True)
# ax3.legend()

# Add overall title
plt.suptitle(f'ESP32 Control System Analysis (Target Velocity: {TARGET_VELOCITY})', fontsize=16, y=1.02)

# Save the plot
plt.savefig(save_path, dpi=300, bbox_inches='tight')
plt.show()

# exit(1)
