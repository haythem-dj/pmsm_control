import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

PI = 3.1415926535

R = 3.26 / 2
L = 3.02e-3 / 2
p = 8
J = 8.75e-6
b = 1e-6
K_t = 0.0498
psi_f = K_t / (1.5 * p)

Kp_d, Ki_d = 5.0, 200.0
Kp_q, Ki_q = 5.0, 200.0
Kp_wm, Ki_wm = 0.5, 20.0
Kp_theta_l = 10

V_MAX = 48
I_MAX = 1.18
GEAR_RATIO = 210

i_d_ref = 0
omega_ref = 20
theta_l_ref = 180
T_l = 0.0


def model(t, y):
    i_a, i_b, i_c, w_m, theta_m, int_i_d, int_i_q, int_w_m = y

    theta_e = p * theta_m
    theta_l = theta_m / GEAR_RATIO

    theta_l_error = theta_l_ref * PI / 180 - theta_l
    w_l_ref_raw = Kp_theta_l * theta_l_error
    w_l_ref = np.clip(w_l_ref_raw, -omega_ref * 2 * PI / 60, omega_ref * 2 * PI / 60)

    w_m_ref = w_l_ref * GEAR_RATIO

    w_m_error = w_m_ref - w_m
    i_q_ref_raw = Kp_wm * w_m_error + int_w_m
    i_q_ref = np.clip(i_q_ref_raw, -I_MAX, I_MAX)

    if i_q_ref != i_q_ref_raw and np.sign(w_m_error) == np.sign(i_q_ref_raw - i_q_ref):
        dint_w_m = 0.0
    else:
        dint_w_m = Ki_wm * w_m_error

    i_alpha = i_a
    i_beta = i_a * np.sqrt(3) / 3 + i_b * 2 * np.sqrt(3) / 3

    i_d = np.cos(theta_e) * i_alpha + np.sin(theta_e) * i_beta
    i_q = -np.sin(theta_e) * i_alpha + np.cos(theta_e) * i_beta

    i_d_error = i_d_ref - i_d
    i_q_error = i_q_ref - i_q

    V_d_raw = Kp_d * i_d_error + int_i_d
    V_q_raw = Kp_q * i_q_error + int_i_q

    V_d = np.clip(V_d_raw, -V_MAX / 2, V_MAX / 2)
    V_q = np.clip(V_q_raw, -V_MAX / 2, V_MAX / 2)

    if V_d != V_d_raw and np.sign(i_d_error) == np.sign(V_d_raw - V_d):
        dint_i_d = 0.0
    else:
        dint_i_d = Ki_d * i_d_error

    if V_q != V_q_raw and np.sign(i_q_error) == np.sign(V_q_raw - V_q):
        dint_i_q = 0.0
    else:
        dint_i_q = Ki_q * i_q_error

    V_alpha = V_d * np.cos(theta_e) - V_q * np.sin(theta_e)
    V_beta = V_d * np.sin(theta_e) + V_q * np.cos(theta_e)

    V_a = V_alpha
    V_b = -0.5 * V_alpha + V_beta * np.sqrt(3) / 2
    V_c = -0.5 * V_alpha - V_beta * np.sqrt(3) / 2

    Te = (
        p
        * psi_f
        * (
            -i_a * np.sin(theta_e)
            - i_b * np.sin(theta_e - 2 * PI / 3)
            - i_c * np.sin(theta_e - 4 * PI / 3)
        )
    )

    di_a = V_a / L - R * i_a / L - p * w_m * psi_f * np.sin(theta_e)
    di_b = V_b / L - R * i_b / L - p * w_m * psi_f * np.sin(theta_e - 2 * PI / 3)
    di_c = V_c / L - R * i_c / L - p * w_m * psi_f * np.sin(theta_e - 4 * PI / 3)
    dw_m = Te / J - T_l / J - b * w_m / J
    dtheta_m = w_m

    return [
        di_a,
        di_b,
        di_c,
        dw_m,
        dtheta_m,
        dint_i_d,
        dint_i_q,
        dint_w_m,
    ]


t_span = [0, 4]
y0 = np.zeros(8)

sol = solve_ivp(model, t_span, y0)

t = sol.t
i_a, i_b, i_c, w_m, theta_m, int_i_d, int_i_q, int_w_m = sol.y
theta_e = p * theta_m
theta_l = (theta_m / GEAR_RATIO) * (180 / PI)
w_l = w_m / GEAR_RATIO

Te = (
    p
    * psi_f
    * (
        -i_a * np.sin(theta_e)
        - i_b * np.sin(theta_e - 2 * PI / 3)
        - i_c * np.sin(theta_e - 4 * PI / 3)
    )
)

plt.figure(figsize=(10, 6))
plt.subplot(2, 2, 1)
plt.plot(sol.t, sol.y[0], label="ia [A]")
plt.plot(sol.t, sol.y[1], label="ib [A]")
plt.plot(sol.t, sol.y[2], label="ic [A]")
plt.title("abc Frame Simulation (np.sinusoidal Excitation)")
plt.ylabel("Currents")
plt.legend()
plt.grid(True)

plt.subplot(2, 2, 2)
plt.plot(sol.t, w_l, "r-", label="low speed [rad/s]")
plt.ylabel("Speed")
plt.xlabel("Time [s]")
plt.legend()
plt.grid(True)

plt.subplot(2, 2, 3)
plt.plot(sol.t, theta_l, "r-", label="load position [°]")
plt.ylabel("Angle")
plt.xlabel("Time [s]")
plt.legend()
plt.grid(True)

plt.subplot(2, 2, 4)
plt.plot(sol.t, Te, "r-", label="Mechanical Torque [Nm]")
plt.ylabel("Torque")
plt.xlabel("Time [s]")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
