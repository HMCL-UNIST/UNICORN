from scipy.optimize import least_squares
import numpy as np

def analyse_tires(model, v_x, v_y, omega, delta, long_acc):
    delta = delta.copy()
    v_y = v_y.copy()
    omega = omega.copy()
    v_x = v_x.copy()
    long_acc = long_acc.copy()
    l_f = model['l_f']
    l_r = model['l_r']
    l_wb = model['l_wb']
    m = model['m']
    h = model['h_cg']
    g_ = 9.81
    # F_zf = m * g_ * l_r / l_wb
    # F_zr = m * g_ * l_f / l_wb
    F_zf = m * (-long_acc * h + g_ * l_r) / l_wb
    F_zr = m * (long_acc * h + g_ * l_f) / l_wb

    alpha_f = -np.arctan((v_y + omega * l_f) / v_x) + delta
    alpha_r = -np.arctan((v_y - omega * l_r) / v_x)

    F_yf = m * l_r * v_x * omega / ((l_r + l_f) * np.cos(delta))
    F_yr = m * l_f * v_x * omega / (l_r + l_f)
    
    # Filtering out diverged data points
    keep_idxs = np.where((abs(omega) <= 5))
    v_x = v_x[keep_idxs]
    v_y = v_y[keep_idxs]
    omega = omega[keep_idxs]
    delta = delta[keep_idxs]
    alpha_f = alpha_f[keep_idxs]
    alpha_r = alpha_r[keep_idxs]
    F_yf = F_yf[keep_idxs]
    F_yr = F_yr[keep_idxs]
    F_zf = F_zf[keep_idxs]
    F_zr = F_zr[keep_idxs]

    keep_idxs = np.where((abs(alpha_f) <= 0.5))
    v_x = v_x[keep_idxs]
    v_y = v_y[keep_idxs]
    omega = omega[keep_idxs]
    delta = delta[keep_idxs]
    alpha_f = alpha_f[keep_idxs]
    alpha_r = alpha_r[keep_idxs]
    F_yf = F_yf[keep_idxs]
    F_yr = F_yr[keep_idxs]
    F_zf = F_zf[keep_idxs]
    F_zr = F_zr[keep_idxs]

    keep_idxs = np.where((abs(alpha_r) <= 0.2))
    v_x = v_x[keep_idxs]
    v_y = v_y[keep_idxs]
    omega = omega[keep_idxs]
    delta = delta[keep_idxs]
    alpha_f = alpha_f[keep_idxs]
    alpha_r = alpha_r[keep_idxs]
    F_yf = F_yf[keep_idxs]
    F_yr = F_yr[keep_idxs]
    F_zf = F_zf[keep_idxs]
    F_zr = F_zr[keep_idxs]
    
    return alpha_f, alpha_r, F_zf, F_zr, F_yf, F_yr


def pacejka_formula(params, alpha, F_z):
    B, C, D, E = params
    y =  F_z * D * np.sin(C*np.arctan(B*alpha - E * (B*alpha -np.arctan(B * alpha))))
    return y

def pacejka_error(params, *args):
    alpha, F_z, F_y = args
    y = pacejka_formula(params, alpha, F_z)
    error = np.sum((y - F_y)**2)
    # square error
    return error


def solve_pacejka(model, v_x, v_y, omega, delta, long_acc):
    alpha_f, alpha_r, F_zf, F_zr, F_yf, F_yr = analyse_tires(model, v_x, v_y, omega, delta, long_acc)
    # front
    start_params_front = model['C_Pf_model']
    sol_f = least_squares(pacejka_error, start_params_front, args=(alpha_f, F_zf, F_yf), bounds=([1.0, 0.1, 0.1, 0.0], [20.0, 20.0, 20.0, 5.0]))
    C_Pf = sol_f.x.tolist()
    C_Pf = [round(x, 4) for x in C_Pf]  # Formatting each element to 4 significant digits

    # rear
    start_params_rear = model['C_Pr_model']
    sol_r = least_squares(pacejka_error, start_params_rear, args=(alpha_r, F_zr, F_yr), bounds=([1.0, 0.1, 0.1, 0.0], [20.0, 20.0, 20.0, 5.0]))
    C_Pr = sol_r.x.tolist()
    C_Pr = [round(x, 4) for x in C_Pr]  # Formatting each element to 4 significant digits
    return C_Pf, C_Pr

