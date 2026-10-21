# Maxon EC-i 30 PMSM Digital Twin & FOC Framework

A high-fidelity digital twin and control simulation framework for a Permanent Magnet Synchronous Motor (PMSM) actuator, featuring cascaded Field-Oriented Control (FOC), Space Vector PWM (SVPWM) via zero-sequence injection, and a realistic $3$-phase inverter bridge.

---

## 🚀 Key Features

- **Cascaded Control Architecture:** Inner current loops ($d-q$ axes) paired with outer speed and position loops. The position controller removes integrators to eliminate overshoot.
- **Advanced PWM & Inverter Simulation:** Supports both standard Sinusoidal PWM (SPWM) and **Space Vector PWM (SVPWM)** using a $48\text{V}$ DC bus.
- **Zero-Sequence Injection:** Implements min-max common-mode offset injection ($\text{offset} = -0.5(V_{max} + V_{min})$) to achieve a $\sim 15\%$ voltage boost and maximize DC bus utilization without complex sector lookups.
- **Multi-Environment Support:** Structured for implementation across Python (numpy/SciPy/Pygame-CE) and MATLAB/Simulink/Simscape.
- **Realistic Plant Modeling:** Incorporates motor parameters ($R, L$), a $210:1$ gearbox ratio, and vector magnitude voltage limiting to prevent overmodulation.

---

## 📐 System Specifications

| Parameter               |  Symbol  |  Value  |     Unit     |
| :---------------------- | :------: | :-----: | :----------: |
| **DC Bus Voltage**      | $V_{dc}$ |  $48$   |  $\text{V}$  |
| **Winding Resistance**  |   $R$    | $1.63$  |   $\Omega$   |
| **Winding Inductance**  |   $L$    | $1.51$  | $\text{mH}$  |
| **Gearbox Ratio**       |   $N$    | $210:1$ |      —       |
| **Switching Frequency** | $f_{sw}$ |  $20$   | $\text{kHz}$ |

---

## ⚙️ Control & Modulation Pipeline

1. **FOC Core:** Computes reference voltages $V_d$ and $V_q$ based on PI current controllers aligned with the rotor's electrical angle ($\theta_e$).
2. **Voltage Limiting:** Applies circular vector magnitude clamping ($\sqrt{V_d^2 + V_q^2}$) to keep voltage commands within the linear modulation boundary ($\frac{V_{dc}}{\sqrt{3}}$ for SVPWM).
3. **Inverse Transforms:** Converts rotating frame voltages back to stationary 2-axis $(\alpha\beta)$ and 3-phase $(abc)$ sinusoidal references.
4. **SVPWM Modulation:** Evaluates instantaneous phase limits and injects the zero-sequence offset:
   $$V_{\text{offset}} = -0.5 \left( \max(V_a, V_b, V_c) + \min(V_a, V_b, V_c) \right)$$
5. **Inverter Bridge:** Compares modulated references against a high-frequency triangular carrier wave to generate $0\text{V} / 48\text{V}$ PWM gate pulses for the 3-phase MOSFET bridge.

---
