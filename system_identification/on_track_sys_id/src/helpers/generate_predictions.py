import numpy as np

def generate_predictions(data, model):
    """
    Generate predicted lateral velocity and yaw rate using Pacejka tire model.

    Args:
        data (numpy.ndarray): Input data array with shape (n_samples, 4).
        model (dict): Dictionary containing vehicle model parameters.

    Returns:
        tuple: Tuple containing predicted lateral velocity and yaw rate arrays.
    """
    
    # Load model parameters
    m = model['m']  # Mass
    I_z = model['I_z']  # Moment of inertia
    l_f = model['l_f']  # Distance from center of mass to front axle
    l_r = model['l_r']  # Distance from center of mass to rear axle
    l_wb = model['l_wb']  # Wheelbase
    h = model['h_cg'] # Distance from center of mass to ground

    g_ = 9.81  # Gravity
    dt = 0.02

    # Pacejka tire model parameters
    C_Pf_model = model['C_Pf_model']
    C_Pr_model = model['C_Pr_model']

    # Extract data
    v_x = data[:, 0]
    v_y = data[:, 1]
    omega = data[:, 2]
    delta = data[:, 3]
    long_acc = data[:, 4]

    timesteps = np.size(v_x)
    
    # Compute vertical forces on front and rear axles
    # F_zf = m * g_ * l_r / l_wb
    # F_zr = m * g_ * l_f / l_wb
    
    # If considering the weight transfer due to longitudinal acceleration
    F_zf = m * (-long_acc * h + g_ * l_r) / (l_r + l_f)
    F_zr = m * (long_acc * h + g_ * l_f) / (l_r + l_f)

    # Extract Pacejka tire model parameters
    B_f, C_f, D_f, E_f = C_Pf_model
    B_r, C_r, D_r, E_r = C_Pr_model

    # Initialize array fors predicted next states
    v_y_next_pred = np.zeros(timesteps - 1)
    omega_next_pred = np.zeros(timesteps - 1)

    for t in range(timesteps - 1):
        # Calculate slip angles
        alpha_f = -np.arctan((v_y[t] + omega[t] * l_f) / v_x[t]) + delta[t]
        alpha_r = -np.arctan((v_y[t] - omega[t] * l_r) / v_x[t])

        # Calculate Pacejka lateral forces
        # F_f = F_zf * D_f * np.sin(C_f * np.arctan(B_f * alpha_f - E_f * (B_f * alpha_f - np.arctan(B_f * alpha_f))))
        # F_r = F_zr * D_r * np.sin(C_r * np.arctan(B_r * alpha_r - E_r * (B_r * alpha_r - np.arctan(B_r * alpha_r))))

        # If considering the weight transfer due to longitudinal acceleration
        F_f = F_zf[t] * D_f * np.sin(C_f * np.arctan(B_f * alpha_f - E_f * (B_f * alpha_f - np.arctan(B_f * alpha_f))))
        F_r = F_zr[t] * D_r * np.sin(C_r * np.arctan(B_r * alpha_r - E_r * (B_r * alpha_r - np.arctan(B_r * alpha_r))))

        # Compute accelerations
        v_y_dot = (1/m) * (F_r + F_f * np.cos(delta[t]) - m * v_x[t] * omega[t])
        omega_dot = (1/I_z) * (F_f * l_f * np.cos(delta[t]) - F_r * l_r)

        # Integrate to get the next predicted v_y and omega
        v_y_next_pred[t] = v_y[t] + v_y_dot * dt
        omega_next_pred[t] = omega[t] + omega_dot * dt

    return v_y_next_pred, omega_next_pred
