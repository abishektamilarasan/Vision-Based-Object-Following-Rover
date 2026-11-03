#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Reads the latest benchmark test results from rover-follower/measurements/
and automatically fills in Section 6.2 and 6.3 of PROJECT_DOCUMENTATION.md!
"""
import os, sys, glob, json, re, io

if sys.stdout.encoding and sys.stdout.encoding.lower() not in ('utf-8', 'utf8'):
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MEASURE_DIR = os.path.join(BASE_DIR, "..", "measurements")
DOC_PATH = os.path.join(BASE_DIR, "..", "PROJECT_DOCUMENTATION.md")


def get_latest_file(pattern):
    files = glob.glob(os.path.join(MEASURE_DIR, pattern))
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def load_json(filepath):
    if not filepath or not os.path.exists(filepath):
        return None
    try:
        with open(filepath, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        print(f"Error loading {filepath}: {e}")
        return None


def main():
    print("=" * 65)
    print("  FILLING BENCHMARK MEASUREMENTS INTO PROJECT DOCUMENTATION")
    print("=" * 65)

    if not os.path.exists(DOC_PATH):
        print(f"Documentation file not found at: {DOC_PATH}")
        return

    with open(DOC_PATH, "r", encoding="utf-8") as f:
        doc_content = f.read()

    # 1. Focal length
    focal_file = get_latest_file("focal_length_*.json")
    focal_data = load_json(focal_file)
    if focal_data:
        fl_val = f"{focal_data.get('focal_length_px', 'N/A')}"
        doc_content = re.sub(
            r'(\|\s*1\s*\|\s*\*\*Camera Focal Length\*\*\s*\|\s*~?502\s*\|\s*)([^\|]+)(\|\s*px\s*\|)',
            rf'\g<1>**{fl_val}** \g<3>',
            doc_content
        )
        print(f"✓ Item 1 Focal Length: {fl_val} px")

    # 2. Latency
    lat_file = get_latest_file("latency_summary_*.json")
    lat_data = load_json(lat_file)
    if lat_data:
        tot = lat_data.get("total", {})
        fg = lat_data.get("frame_grab", {})
        det = lat_data.get("aruco_detect", {})
        kf = lat_data.get("kalman_step", {})
        nav = lat_data.get("nav_compute", {})
        cmd = lat_data.get("cmd_dispatch", {})

        tot_str = f"**{tot.get('mean_ms', 0):.1f} ± {tot.get('std_ms', 0):.1f}**"
        doc_content = re.sub(
            r'(\|\s*4\s*\|\s*\*\*Total Closed-Loop Latency\*\*\s*\|\s*[^\|]+\|\s*)([^\|]+)(\|\s*ms\s*\|)',
            rf'\g<1>{tot_str} \g<3>',
            doc_content
        )
        if fg:
            doc_content = re.sub(
                r'(\|\s*4a\s*\|\s*↳ Frame Grab\s*\|\s*[^\|]+\|\s*)([^\|]+)(\|\s*ms\s*\|)',
                rf"\g<1>{fg.get('mean_ms', 0):.1f} ± {fg.get('std_ms', 0):.1f} \g<3>",
                doc_content
            )
        if det:
            doc_content = re.sub(
                r'(\|\s*4b\s*\|\s*↳ ArUco Detection\s*\|\s*[^\|]+\|\s*)([^\|]+)(\|\s*ms\s*\|)',
                rf"\g<1>{det.get('mean_ms', 0):.1f} ± {det.get('std_ms', 0):.1f} \g<3>",
                doc_content
            )
        if kf:
            doc_content = re.sub(
                r'(\|\s*4c\s*\|\s*↳ Kalman Filter Step\s*\|\s*[^\|]+\|\s*)([^\|]+)(\|\s*ms\s*\|)',
                rf"\g<1>{kf.get('mean_ms', 0):.2f} ± {kf.get('std_ms', 0):.2f} \g<3>",
                doc_content
            )
        if nav:
            doc_content = re.sub(
                r'(\|\s*4d\s*\|\s*↳ PID \+ Nav Compute\s*\|\s*[^\|]+\|\s*)([^\|]+)(\|\s*ms\s*\|)',
                rf"\g<1>{nav.get('mean_ms', 0):.2f} ± {nav.get('std_ms', 0):.2f} \g<3>",
                doc_content
            )
        if cmd:
            doc_content = re.sub(
                r'(\|\s*4e\s*\|\s*↳ Command Dispatch\s*\|\s*[^\|]+\|\s*)([^\|]+)(\|\s*ms\s*\|)',
                rf"\g<1>{cmd.get('mean_ms', 0):.1f} ± {cmd.get('std_ms', 0):.1f} \g<3>",
                doc_content
            )
        print(f"✓ Item 4 Latency: Total = {tot.get('mean_ms', 0):.1f} ms")

    # 3. FPS
    fps_file = get_latest_file("fps_*.json")
    fps_data = load_json(fps_file)
    if fps_data:
        fps_val = f"**{fps_data.get('fps', 0):.1f}**"
        doc_content = re.sub(
            r'(\|\s*5a\s*\|\s*\*\*FPS \(ArUco Mode\)\*\*\s*\|\s*[^\|]+\|\s*)([^\|]+)(\|\s*FPS\s*\|)',
            rf'\g<1>{fps_val} \g<3>',
            doc_content
        )
        print(f"✓ Item 5 FPS: {fps_val}")

    # 4. Kalman Benefit
    kb_file = get_latest_file("kalman_benefit_*.json")
    kb_data = load_json(kb_file)
    if kb_data:
        raw_jit = f"{kb_data.get('raw_mean_jitter_px', 0):.2f}"
        filt_jit = f"{kb_data.get('filtered_mean_jitter_px', 0):.2f}"
        red_pct = f"**{kb_data.get('jitter_reduction_pct', 0):.1f}%**"

        doc_content = re.sub(
            r'(\|\s*6a\s*\|\s*\*\*Kalman Jitter \(Raw\)\*\*\s*\|\s*—\s*\|\s*)([^\|]+)(\|\s*px/frame\s*\|)',
            rf'\g<1>{raw_jit} \g<3>',
            doc_content
        )
        doc_content = re.sub(
            r'(\|\s*6b\s*\|\s*\*\*Kalman Jitter \(Filtered\)\*\*\s*\|\s*—\s*\|\s*)([^\|]+)(\|\s*px/frame\s*\|)',
            rf'\g<1>{filt_jit} \g<3>',
            doc_content
        )
        doc_content = re.sub(
            r'(\|\s*6c\s*\|\s*\*\*Jitter Reduction\*\*\s*\|\s*—\s*\|\s*)([^\|]+)(\|\s*%\s*\|)',
            rf'\g<1>{red_pct} \g<3>',
            doc_content
        )
        print(f"✓ Item 6 Kalman Jitter Reduction: {red_pct}")

    # 5. Step Response
    sr_file = get_latest_file("step_response_metrics_*.json")
    sr_data = load_json(sr_file)
    if sr_data:
        tr = f"{sr_data.get('rise_time_s', 'N/A')}"
        ts = f"{sr_data.get('settling_time_s', 'N/A')}"
        os_pct = f"{sr_data.get('overshoot_pct', 'N/A')}"
        ss_err = f"{sr_data.get('steady_state_error', 'N/A')}"

        doc_content = re.sub(
            r'(\|\s*7a\s*\|\s*\*\*Step Response: Rise Time\*\*\s*\|\s*—\s*\|\s*)([^\|]+)(\|\s*s\s*\|)',
            rf'\g<1>{tr} \g<3>',
            doc_content
        )
        doc_content = re.sub(
            r'(\|\s*7b\s*\|\s*\*\*Step Response: Settling Time\*\*\s*\|\s*—\s*\|\s*)([^\|]+)(\|\s*s\s*\|)',
            rf'\g<1>{ts} \g<3>',
            doc_content
        )
        doc_content = re.sub(
            r'(\|\s*7c\s*\|\s*\*\*Step Response: Overshoot\*\*\s*\|\s*—\s*\|\s*)([^\|]+)(\|\s*%\s*\|)',
            rf'\g<1>{os_pct} \g<3>',
            doc_content
        )
        doc_content = re.sub(
            r'(\|\s*7d\s*\|\s*\*\*Step Response: SS Error\*\*\s*\|\s*—\s*\|\s*)([^\|]+)(\|\s*—\s*\|)',
            rf'\g<1>{ss_err} \g<3>',
            doc_content
        )
        print(f"✓ Item 7 Step Response: Rise={tr}s, Settling={ts}s, Overshoot={os_pct}%")

    # 6. Breakaway PWM
    bp_file = get_latest_file("breakaway_pwm_*.json")
    bp_data = load_json(bp_file)
    if bp_data:
        bp_fwd = str(bp_data.get("forward", "85"))
        doc_content = re.sub(
            r'(\|\s*8\s*\|\s*\*\*Breakaway PWM\*\*\s*\|\s*85\s*\|\s*)([^\|]+)(\|\s*PWM\s*\(0–255\)\s*\|)',
            rf'\g<1>**{bp_fwd}** \g<3>',
            doc_content
        )
        print(f"✓ Item 8 Breakaway PWM: {bp_fwd}")

    # 7. Wi-Fi RTT
    wifi_file = get_latest_file("wifi_rtt_*.json")
    wifi_data = load_json(wifi_file)
    if wifi_data:
        ctrl = wifi_data.get("control", {})
        rtt_str = f"**{ctrl.get('mean_ms', 0):.1f} ± {ctrl.get('std_ms', 0):.1f}**"
        doc_content = re.sub(
            r'(\|\s*17\s*\|\s*\*\*Wi-Fi RTT \(/control\)\*\*\s*\|\s*~?12\s*\|\s*)([^\|]+)(\|\s*ms\s*\|)',
            rf'\g<1>{rtt_str} \g<3>',
            doc_content
        )
        print(f"✓ Item 17 Wi-Fi RTT: {rtt_str} ms")

    with open(DOC_PATH, "w", encoding="utf-8") as f:
        f.write(doc_content)

    print("\n✓ Documentation updated successfully!")


if __name__ == "__main__":
    main()
