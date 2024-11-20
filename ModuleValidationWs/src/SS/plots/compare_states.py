import pandas as pd
import matplotlib.pyplot as plt
import os


# Load CSV files
trans_real_path = "../SimData/RealStateSpace_trans.csv"
rot_real_path = "../SimData/RealStateSpace_rot.csv"
trans_est_path = "../SimData/TranslationalEstimation.csv"
rot_est_path = "../SimData/RotationalEstimation.csv"

# Check if all required CSV files exist
if not all(os.path.exists(path) for path in [trans_real_path, rot_real_path, trans_est_path, rot_est_path]):
    print("One or more CSV files are missing. Please check the paths.")
    exit()

# Load data
trans_real = pd.read_csv(trans_real_path)
rot_real = pd.read_csv(rot_real_path)
trans_est = pd.read_csv(trans_est_path)
rot_est = pd.read_csv(rot_est_path)

is_luenberger = "ddist_x" in trans_est.columns
if is_luenberger:
    numeric_columns_trans = [
        'dp_x', 'dp_y', 'dp_z', 'ddp_x', 'ddp_y', 'ddp_z', 'w_hat_x', 'w_hat_y', 'w_hat_z'
    ]
    numeric_columns_rot = [
        'dq_x', 'dq_y', 'dq_z', 'domega_x', 'domega_y', 'domega_z', 'w_hat_x', 'w_hat_y', 'w_hat_z'
    ]
else:
    numeric_columns_trans = [
        'dp_x', 'dp_y', 'dp_z', 'ddp_x', 'ddp_y', 'ddp_z', 'w_hat_x', 'w_hat_y', 'w_hat_z'
    ]
    numeric_columns_rot = [
        'dq_x', 'dq_y', 'dq_z', 'domega_x', 'domega_y', 'domega_z', 'w_hat_x', 'w_hat_y', 'w_hat_z'
    ]

def ensure_numeric(df, columns):
    for col in columns:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors='coerce')

ensure_numeric(trans_real, numeric_columns_trans)
ensure_numeric(trans_est, numeric_columns_trans)
ensure_numeric(rot_real, numeric_columns_rot)
ensure_numeric(rot_est, numeric_columns_rot)


# Plot comparison for Translational States (Velocity and Acceleration)
plt.figure("Translational State Comparison", figsize=(12, 8))
plt.suptitle("Translational State Comparison")

# Velocity (dp) comparison
plt.subplot(3, 1, 1)
plt.plot(trans_real['dp_x'], label='Real dp_x', linestyle='--')
plt.plot(trans_est['dp_x'], label='Estimation dp_x')
plt.plot(trans_real['dp_y'], label='Real dp_y', linestyle='--')
plt.plot(trans_est['dp_y'], label='Estimation dp_y')
plt.plot(trans_real['dp_z'], label='Real dp_z', linestyle='--')
plt.plot(trans_est['dp_z'], label='Estimation dp_z')
plt.legend()
plt.title("Velocity (dp) Comparison")
plt.xlabel("Time Step")
plt.ylabel("Velocity (m/s)")

# Acceleration (ddp) comparison
plt.subplot(3, 1, 2)
plt.plot(trans_real['ddp_x'], label='Real ddp_x', linestyle='--')
plt.plot(trans_est['ddp_x'], label='Estimation ddp_x')
plt.plot(trans_real['ddp_y'], label='Real ddp_y', linestyle='--')
plt.plot(trans_est['ddp_y'], label='Estimation ddp_y')
plt.plot(trans_real['ddp_z'], label='Real ddp_z', linestyle='--')
plt.plot(trans_est['ddp_z'], label='Estimation ddp_z')
plt.legend()
plt.title("Acceleration (ddp) Comparison")
plt.xlabel("Time Step")
plt.ylabel("Acceleration (m/s²)")

# Translational disturbance estimation
plt.subplot(3, 1, 3)
plt.plot(trans_est['w_hat_x'], label='Disturbance w_hat_x')
plt.plot(trans_est['w_hat_y'], label='Disturbance w_hat_y')
plt.plot(trans_est['w_hat_z'], label='Disturbance w_hat_z')
plt.legend()
plt.title("Translational Disturbance Estimation")
plt.xlabel("Time Step")
plt.ylabel("Disturbance")

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()

# Plot comparison for Rotational States (Quaternion Derivative and Angular Acceleration)
plt.figure("Rotational State Comparison", figsize=(12, 8))
plt.suptitle("Rotational State Comparison")

# Quaternion derivative (dq) comparison
plt.subplot(3, 1, 1)
plt.plot(rot_real['dq_x'], label='Real dq_x', linestyle='--')
plt.plot(rot_est['dq_x'], label='Estimation dq_x')
plt.plot(rot_real['dq_y'], label='Real dq_y', linestyle='--')
plt.plot(rot_est['dq_y'], label='Estimation dq_y')
plt.plot(rot_real['dq_z'], label='Real dq_z', linestyle='--')
plt.plot(rot_est['dq_z'], label='Estimation dq_z')
plt.legend()
plt.title("Quaternion derivative (dq) Comparison")
plt.xlabel("Time Step")
plt.ylabel("dq")

# Angular acceleration (domega) comparison
plt.subplot(3, 1, 2)
plt.plot(rot_real['domega_x'], label='Real domega_x', linestyle='--')
plt.plot(rot_est['domega_x'], label='Estimation domega_x')
plt.plot(rot_real['domega_y'], label='Real domega_y', linestyle='--')
plt.plot(rot_est['domega_y'], label='Estimation domega_y')
plt.plot(rot_real['domega_z'], label='Real domega_z', linestyle='--')
plt.plot(rot_est['domega_z'], label='Estimation domega_z')
plt.legend()
plt.title("Angular Acceleration (domega) Comparison")
plt.xlabel("Time Step")
plt.ylabel("Angular Acceleration (rad/s²)")

# Rotational disturbance estimation
plt.subplot(3, 1, 3)
plt.plot(rot_est['w_hat_x'], label='Disturbance w_hat_x')
plt.plot(rot_est['w_hat_y'], label='Disturbance w_hat_y')
plt.plot(rot_est['w_hat_z'], label='Disturbance w_hat_z')
plt.legend()
plt.title("Rotational Disturbance Estimation")
plt.xlabel("Time Step")
plt.ylabel("Disturbance")

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()


# Disrutbances comparison
plt.figure("Translational and Rotational Disturbance Comparison", figsize=(12, 8))
plt.suptitle("Disturbance Comparison")

plt.plot(trans_est['w_hat_x'], label='Disturbance w_hat_x')
plt.plot(trans_est['w_hat_y'], label='Disturbance w_hat_y')
plt.plot(trans_est['w_hat_z'], label='Disturbance w_hat_z')
plt.plot(rot_est['w_hat_x'], label='Disturbance w_hat_x')
plt.plot(rot_est['w_hat_y'], label='Disturbance w_hat_y')
plt.plot(rot_est['w_hat_z'], label='Disturbance w_hat_z')
plt.legend()
plt.title("Disturbance Estimation")
plt.xlabel("Time Step")
plt.ylabel("Disturbance")

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()


