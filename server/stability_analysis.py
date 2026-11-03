#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rover Control Systems Stability Analysis & Steady-State Kalman Observer Tool
Pure NumPy implementation:
- Solves Discrete Riccati Equation (DARE) via iterative fixed-point Riccati recursion
- Computes frequency response Bode margins using complex polynomial evaluation
- Simulates discrete time-domain step responses for P-only vs PID
"""
import io, sys, os, time, json
import numpy as np

if sys.stdout.encoding and sys.stdout.encoding.lower() not in ('utf-8', 'utf8'):
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config

RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "measurements")
os.makedirs(RESULTS_DIR, exist_ok=True)


def solve_kalman_steady_state():
    print("\n" + "="*65)
    print("  1. DISCRETE KALMAN FILTER STEADY-STATE ANALYSIS (DARE)")
    print("="*65)
    
    dt = 0.05  # nominal 20 FPS
    F = np.eye(8, dtype=np.float64)
    for i in range(4):
        F[i, i + 4] = dt

    H = np.zeros((4, 8), dtype=np.float64)
    for i in range(4):
        H[i, i] = 1.0

    q_pos = 1.0
    q_vel = 5.0
    q_p = (q_pos * dt) ** 2
    q_v = (q_vel * dt) ** 2

    Q = np.zeros((8, 8), dtype=np.float64)
    for i in range(4):
        Q[i, i] = q_p
        Q[i + 4, i + 4] = q_v

    R = np.eye(4, dtype=np.float64) * (2.0 ** 2)

    print(f"Sampling Period dt : {dt:.3f} s")
    print(f"Process Noise Q    : diag(q_pos={q_p:.6f}, q_vel={q_v:.6f})")
    print(f"Measurement Noise R: diag(r={R[0,0]:.2f})")

    # Fixed point Riccati iteration: P_{k+1} = F [ P_k - P_k H^T (H P_k H^T + R)^-1 H P_k ] F^T + Q
    P = np.eye(8, dtype=np.float64) * 10.0
    for iteration in range(1000):
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        P_post = (np.eye(8) - K @ H) @ P
        P_next = F @ P_post @ F.T + Q
        if np.max(np.abs(P_next - P)) < 1e-9:
            P = P_next
            break
        P = P_next

    # Final Steady State Kalman Gain: K_ss = P H^T (H P H^T + R)^-1
    S = H @ P @ H.T + R
    K_ss = P @ H.T @ np.linalg.inv(S)

    print("\nSteady-State Prior Covariance P_prior diagonal:")
    print(np.round(np.diag(P), 4))
    
    print("\nSteady-State Kalman Gain Matrix K_ss (dim 8x4):")
    for row in range(8):
        row_str = "  ".join([f"{val:+8.4f}" for val in K_ss[row, :]])
        state_name = ["cx", "cy", "w", "h", "vx", "vy", "vw", "vh"][row]
        print(f"  {state_name:<3}: [ {row_str} ]")

    k_pos = K_ss[0, 0]
    k_vel = K_ss[4, 0]
    print(f"\n1D Position Correction Gain (K_pos): {k_pos:.4f}")
    print(f"1D Velocity Correction Gain (K_vel): {k_vel:.4f}")

    return {
        "K_pos": round(float(k_pos), 4),
        "K_vel": round(float(k_vel), 4),
        "P_diag": [round(float(x), 4) for x in np.diag(P)],
        "K_ss": [[round(float(val), 4) for val in row] for row in K_ss]
    }


def analyze_frequency_response(delays=[0.03, 0.05, 0.08, 0.12, 0.20]):
    print("\n" + "="*65)
    print("  2. CLOSED-LOOP STABILITY ANALYSIS ACROSS TRANSPORT DELAYS")
    print("="*65)

    Kp = config.PID_ANGULAR_KP       # 0.35
    Ki = config.PID_ANGULAR_KI       # 0.001
    Kd = config.PID_ANGULAR_KD       # 0.06
    alpha = getattr(config, "ARUCO_SMOOTH_ALPHA", 0.45)
    Ts = 0.05

    Km = 0.023                       # Motor rad/(s*PWM)
    tau_m = 0.10                     # Motor time constant (s)
    K_scale = config.MAX_SPEED * 0.45 # 170 * 0.45 = 76.5
    # Normalization gain: error_normalized = delta_x_px / (frame_center_x - deadzone_px)
    frame_center_x = config.FRAME_WIDTH / 2.0
    deadzone_px = float(getattr(config, "ARUCO_DEADZONE_X", 35))
    K_sensor = 1.0 / (frame_center_x - deadzone_px) # ~ 1 / 285 px^-1

    # Camera mapping: delta_x_px = f * theta
    K_cam = 502.0                    # px/rad
    tau_s = ((1.0 - alpha) / alpha) * Ts # EWMA continuous equivalent tau (~0.061s)

    # Static open-loop loop gain (dimensionless):
    K_loop = K_sensor * K_scale * Km * K_cam    # (1/285) * 76.5 * 0.023 * 502 ≈ 3.10

    print(f"Static Loop Gain K_total : {K_loop:.2f}")
    print(f"EWMA Filter Time Const   : {tau_s*1000:.1f} ms")
    print(f"Motor Mechanical Time Const: {tau_m*1000:.1f} ms\n")
    print(f"{'Delay (ms)':<12} {'Gain Margin (dB)':<18} {'Phase Margin (deg)':<20} {'Crossover w_c (rad/s)':<24} {'Status'}")
    print("-" * 82)

    w = np.logspace(-1, 3, 20000)
    s = 1j * w

    # Evaluate base G_OL(s) without delay
    num_base = (Kd * s**2 + Kp * s + Ki) * K_loop
    den_base = (s**2) * (tau_s * s + 1.0) * (tau_m * s + 1.0)
    G_base = num_base / den_base

    results = []

    for Td in delays:
        delay_factor = np.exp(-s * Td)
        G_ol = G_base * delay_factor

        mag = np.abs(G_ol)
        mag_db = 20 * np.log10(mag)
        phase_deg = np.degrees(np.unwrap(np.angle(G_ol)))

        # Gain Crossover Frequency (where mag_db == 0)
        idx_0db = np.where(np.diff(np.sign(mag_db)))[0]
        wc = 0.0
        pm_deg = float('inf')
        if len(idx_0db) > 0:
            idx = idx_0db[0]
            wc = w[idx]
            # Phase Margin = 180 + Phase at wc
            pm_deg = 180.0 + phase_deg[idx]
            while pm_deg > 180: pm_deg -= 360
            while pm_deg < -180: pm_deg += 360

        # Phase Crossover Frequency (where phase == -180)
        idx_180 = np.where(np.diff(np.sign(phase_deg - (-180.0))))[0]
        gm_db = float('inf')
        if len(idx_180) > 0:
            idx = idx_180[0]
            gm_db = -mag_db[idx]

        is_stable = bool((pm_deg > 0) and (gm_db > 0 or np.isinf(gm_db)))
        status_str = "STABLE" if is_stable else "UNSTABLE"

        results.append({
            "delay_ms": int(Td * 1000),
            "gain_margin_db": round(float(gm_db), 2) if not np.isinf(gm_db) else "inf",
            "phase_margin_deg": round(float(pm_deg), 2) if not np.isinf(pm_deg) else "N/A",
            "crossover_wc_rad_s": round(float(wc), 2),
            "is_stable": bool(is_stable)
        })

        gm_str = f"{gm_db:.2f}" if not np.isinf(gm_db) else "Inf"
        pm_str = f"{pm_deg:.2f}" if not np.isinf(pm_deg) else "N/A"
        print(f"{int(Td*1000):<12} {gm_str:<18} {pm_str:<20} {wc:<24.2f} {status_str}")

    return results


def simulate_p_vs_pid_step():
    print("\n" + "="*65)
    print("  3. DISCRETE STEP RESPONSE: P-ONLY vs FULL PID")
    print("="*65)

    dt = 0.02
    total_time = 3.0
    steps = int(total_time / dt)
    t = np.linspace(0, total_time, steps)

    def run_sim(kp, ki, kd, use_kd=True):
        tau_m = 0.10
        Km = 0.023
        K_scale = config.MAX_SPEED * 0.45
        alpha = 0.45
        delay_steps = int(0.08 / dt) # 80ms transport delay

        theta = 0.0
        omega = 0.0
        smooth_u = 0.0
        integral = 0.0
        prev_error = 0.0

        u_buffer = [0.0] * (delay_steps + 1)
        resp = []

        setpoint = 1.0 # unit step

        for step in range(steps):
            error = setpoint - theta
            integral += error * dt
            integral = max(min(integral, 10.0), -10.0)
            derivative = (error - prev_error) / dt if use_kd else 0.0
            prev_error = error

            u_raw = kp * error + ki * integral + kd * derivative
            smooth_u = alpha * u_raw + (1.0 - alpha) * smooth_u

            u_buffer.append(smooth_u)
            u_delayed = u_buffer.pop(0)

            d_omega = (Km * K_scale * u_delayed - omega) / tau_m
            omega += d_omega * dt

            theta += omega * (dt / 1.0)
            resp.append(theta)

        resp = np.array(resp)
        peak_val = np.max(resp)
        final_val = resp[-1]
        overshoot = max(0.0, (peak_val - final_val) / final_val * 100.0)

        idx_10 = np.where(resp >= 0.1 * final_val)[0]
        idx_90 = np.where(resp >= 0.9 * final_val)[0]
        tr = (t[idx_90[0]] - t[idx_10[0]]) if (len(idx_10) and len(idx_90)) else 0.0

        settled_idx = np.where(np.abs(resp - final_val) > 0.05 * final_val)[0]
        ts = t[settled_idx[-1]] if len(settled_idx) else 0.0
        ss_error = abs(1.0 - final_val)

        return {
            "rise_time_s": round(float(tr), 3),
            "settling_time_s": round(float(ts), 3),
            "overshoot_pct": round(float(overshoot), 2),
            "ss_error": round(float(ss_error), 4)
        }

    m_pid = run_sim(config.PID_ANGULAR_KP, config.PID_ANGULAR_KI, config.PID_ANGULAR_KD, True)
    m_ponly = run_sim(config.PID_ANGULAR_KP, 0.0, 0.0, False)

    print(f"{'Metric':<25} {'P-Only (Kp=0.35)':<20} {'Full PID (Kp=0.35, Ki=0.001, Kd=0.06)'}")
    print("-" * 75)
    print(f"{'Rise Time (10%->90%)':<25} {m_ponly['rise_time_s']:<20.3f} {m_pid['rise_time_s']:.3f} s")
    print(f"{'Settling Time (5%)':<25} {m_ponly['settling_time_s']:<20.3f} {m_pid['settling_time_s']:.3f} s")
    print(f"{'Overshoot':<25} {m_ponly['overshoot_pct']:<20.2f} {m_pid['overshoot_pct']:.2f} %")
    print(f"{'Steady-State Error':<25} {m_ponly['ss_error']:<20.4f} {m_pid['ss_error']:.4f}")

    return {"p_only": m_ponly, "pid": m_pid}


def main():
    print("="*65)
    print("  ROVER CONTROL SYSTEMS RIGOR & STABILITY EVALUATION")
    print("="*65)

    kalman_data = solve_kalman_steady_state()
    stability_data = analyze_frequency_response()
    comparison_data = simulate_p_vs_pid_step()

    output = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "kalman_steady_state": kalman_data,
        "stability_margins": stability_data,
        "baseline_comparison": comparison_data
    }

    ts = time.strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(RESULTS_DIR, f"stability_analysis_{ts}.json")
    with open(out_path, "w") as f:
        json.dump(output, f, indent=2)

    print("\n" + "="*65)
    print(f"  ✓ Analysis successfully written to: {out_path}")
    print("="*65)


if __name__ == "__main__":
    main()
