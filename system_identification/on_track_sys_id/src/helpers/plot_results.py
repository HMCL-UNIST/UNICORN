import matplotlib.pyplot as plt
import numpy as np
from helpers.pacejka_formula import pacejka_formula

def plot_results(model, v_x, v_y, omega, delta, C_Pf_identified, C_Pr_identified, iteration):
    """
    Plot system identification results.

    Plots the system identification results for front and rear tires after each iteration of the training.
    """
    C_Pf_model = model['C_Pf_model']
    C_Pr_model = model['C_Pr_model']
    
    C_Pf_ref = model['C_Pf_ref']
    C_Pr_ref = model['C_Pr_ref']
    
    l_f = model['l_f']
    l_r = model['l_r']
    l_wb = model['l_wb']
    m = model['m']
    g_ = 9.81
    
    # Compute vertical forces on front and rear tires
    F_zf = m * g_ * l_r / l_wb
    F_zr = m * g_ * l_f / l_wb

    # Compute lateral forces on front and rear tires
    F_yf = m * l_r * v_x * omega / ((l_r + l_f) * np.cos(delta))
    F_yr = m * l_f * v_x * omega / (l_r + l_f)

    # Compute slip angles for front and rear tires
    alpha_f = -np.arctan((v_y + l_f * omega) / v_x) + delta
    alpha_r = -np.arctan((v_y - l_r * omega) / v_x)
    
    # Plotting
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8,6))
    ax1.scatter(alpha_f, F_yf, color="Green", alpha=0.95, label='Generated Data', s=0.5 )
    ax2.scatter(alpha_r, F_yr, color="Green", alpha=0.95, label='Generated Data', s=0.5 )

    # Plot reference, identified, and prior models
    alpha_space = np.linspace(-0.2, 0.2, 100)
    fit_f = pacejka_formula(C_Pf_ref, alpha_space, F_zf)
    fit_r = pacejka_formula(C_Pr_ref, alpha_space, F_zr)

    ax1.plot(alpha_space, fit_f, 'Black', label='Reference Model')
    ax2.plot(alpha_space, fit_r, 'Black', label='Reference Model')

    fit_f = pacejka_formula(C_Pf_identified, alpha_space, F_zf)
    fit_r = pacejka_formula(C_Pr_identified, alpha_space, F_zr)

    ax1.plot(alpha_space, fit_f, 'Blue', label='Identified Model')
    ax2.plot(alpha_space, fit_r, 'Blue', label='Identified Model')

    fit_f = pacejka_formula(C_Pf_model, alpha_space, F_zf)
    fit_r = pacejka_formula(C_Pr_model, alpha_space, F_zr)

    ax1.plot(alpha_space, fit_f, 'Red', label='Prior Model')
    ax2.plot(alpha_space, fit_r, 'Red', label='Prior Model')
    
    # Formatting
    ax1.set_title('Front tires')
    ax2.set_title('Rear tires')
    fig.suptitle(f'System Identification Results After Iteration {iteration}', fontsize=16)
    for ax in [ax1, ax2]:
        ax.set_xlabel(r'$\alpha_f$ [rad]')
        ax.set_ylabel(r'$F_y$ [N]')
        ax.grid()
        ax.legend(loc='best')
        ax.set_xlim([-0.2, 0.2])
        ax.set_ylim([-15, 15])
    plt.tight_layout()
    plt.show()
