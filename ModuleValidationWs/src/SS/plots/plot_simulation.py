import matplotlib
matplotlib.use('Qt5Agg')  # Set the backend to Qt5 for interactive plots

import pandas as pd
import matplotlib.pyplot as plt
import os

# Print current working directory
print("Current working directory:", os.getcwd())

# Paths to data files
trans_data_path = "../SimData/RealStateSpace_trans.csv"
rot_data_path = "../SimData/RealStateSpace_rot.csv"
control_data_path = "../SimData/ControlData.csv"

# Check if files exist
if os.path.exists(trans_data_path) and os.path.exists(rot_data_path) and os.path.exists(control_data_path):
    print("CSV files found:")
    print(trans_data_path)
    print(rot_data_path)
    print(control_data_path)
else:
    print("CSV files not found. Please check the paths.")
    exit()

# Load data
trans_data = pd.read_csv(trans_data_path)
rot_data = pd.read_csv(rot_data_path)
control_data = pd.read_csv(control_data_path)

# Ensure numeric data
control_data['Tauu_x'] = pd.to_numeric(control_data['Tauu_x'], errors='coerce')
control_data['Tauu_y'] = pd.to_numeric(control_data['Tauu_y'], errors='coerce')
control_data['Tauu_z'] = pd.to_numeric(control_data['Tauu_z'], errors='coerce')
control_data['Fu'] = pd.to_numeric(control_data['Fu'], errors='coerce')

# First window for Position, Velocity, and Orientation
plt.figure("Kinematic Data", figsize=(10, 12))

# Position (p_x, p_y, p_z)
plt.subplot(5, 1, 1)
plt.plot(trans_data['p_x'], label='p_x')
plt.plot(trans_data['p_y'], label='p_y')
plt.plot(trans_data['p_z'], label='p_z')
plt.title("Position (p)")
plt.ylabel("Position (m)")
plt.legend()

# Velocity (dp_x, dp_y, dp_z)
plt.subplot(5, 1, 2)
plt.plot(trans_data['dp_x'], label='dp_x')
plt.plot(trans_data['dp_y'], label='dp_y')
plt.plot(trans_data['dp_z'], label='dp_z')
plt.title("Velocity (dp)")
plt.ylabel("Velocity (m/s)")
plt.legend()

# Linear Acceleration (ddp_x, ddp_y, ddp_z)
plt.subplot(5, 1, 3)
plt.plot(trans_data['ddp_x'], label='ddp_x')
plt.plot(trans_data['ddp_y'], label='ddp_y')
plt.plot(trans_data['ddp_z'], label='ddp_z')
plt.title("Linear Acceleration (ddp)")
plt.ylabel("Acceleration (m/s²)")
plt.legend()

# Orientation (Quaternion)
plt.subplot(5, 1, 4)
plt.plot(rot_data['q_w'], label='q_w')
plt.plot(rot_data['q_x'], label='q_x')
plt.plot(rot_data['q_y'], label='q_y')
plt.plot(rot_data['q_z'], label='q_z')
plt.title("Orientation (Quaternion)")
plt.ylabel("Quaternion")
plt.legend()

# Rotational Acceleration (alpha_x, alpha_y, alpha_z)
plt.subplot(5, 1, 5)
plt.plot(rot_data['domega_x'], label='domega_x')
plt.plot(rot_data['domega_y'], label='domega_y')
plt.plot(rot_data['domega_z'], label='domega_z')
plt.title("Rotational Acceleration")
plt.ylabel("Angular Acceleration (rad/s²)")
plt.legend()

plt.tight_layout()

# Second window for Control Data
plt.figure("Control Data", figsize=(8, 6))

# Control Output (Tauu_x, Tauu_y, Tauu_z, Fu)
plt.plot(control_data['Tauu_x'], label='Tauu_x')
plt.plot(control_data['Tauu_y'], label='Tauu_y')
plt.plot(control_data['Tauu_z'], label='Tauu_z')
plt.plot(control_data['Fu'], label='Fu')
plt.title("Control Output")
plt.xlabel("Time step")
plt.ylabel("Control")
plt.legend()

plt.tight_layout()
plt.show()
