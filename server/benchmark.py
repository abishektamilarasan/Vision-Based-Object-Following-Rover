#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import io, sys
if sys.stdout.encoding and sys.stdout.encoding.lower() not in ('utf-8', 'utf8'):
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')
"""
======================================================================
  ROVER BENCHMARK & INSTRUMENTATION SUITE
  Collects REAL measured values for the project documentation.
  Run from:  rover-follower/server/
  Usage:     python benchmark.py --test <test_name>
======================================================================

Available tests:
  focal_length      Item 1:  Camera focal length calibration
  latency           Item 4:  Pipeline stage latency profiling (1000 frames)
  fps               Item 5:  Measured FPS over 60 seconds
  kalman_benefit    Item 6:  Raw vs filtered position comparison
  step_response     Item 7:  Following step response (MOST IMPORTANT)
  breakaway_pwm     Item 8:  Motor breakaway PWM sweep
  wheel_speed       Item 9:  Timed wheel speed at specific PWMs
  yaw_rate          Item 10: Maximum yaw rate (360° spin timing)
  occlusion         Item 13: Occlusion recovery test
  coasting          Item 14: Coasting distance measurement helper
  watchdog          Item 15: Watchdog stop time measurement
  wifi_rtt          Item 17: Wi-Fi command round-trip time
  distance_cal      Item 3:  Distance estimation accuracy
  all               Runs: focal_length, latency, fps, wifi_rtt (non-motor tests)
"""

import argparse
import csv
import json
import os
import sys
import time
import statistics
import numpy as np

# Add parent directory for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config

# ─── Output directory for measurements ───
RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "measurements")
os.makedirs(RESULTS_DIR, exist_ok=True)


def timestamp_filename(prefix, ext="csv"):
    """Generate a timestamped filename for results."""
    ts = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join(RESULTS_DIR, f"{prefix}_{ts}.{ext}")


# ═══════════════════════════════════════════════════════════════
# Item 1: Camera Focal Length Calibration
# ═══════════════════════════════════════════════════════════════
def test_focal_length():
    """
    Place an ArUco marker at a KNOWN distance from the camera.
    Measures the marker's pixel width, then computes:
        f = (pixel_width × real_distance) / real_marker_size
    
    You will be prompted to enter:
      - Actual marker size in cm (measure with ruler)
      - Distance from camera to marker in cm (measure with tape)
    """
    import cv2
    from aruco_detector import ArucoDetector
    from frame_grabber import FrameGrabber

    print("\n" + "="*60)
    print("  ITEM 1: CAMERA FOCAL LENGTH CALIBRATION")
    print("="*60)
    
    marker_size_cm = float(input("\nEnter the MEASURED marker size in cm (side length): "))
    distance_cm = float(input("Enter the MEASURED distance from camera to marker in cm: "))
    
    print(f"\nConfiguration: marker={marker_size_cm}cm, distance={distance_cm}cm")
    print("Starting camera... Hold the marker steady.")
    
    grabber = FrameGrabber()
    grabber.start()
    detector = ArucoDetector()
    
    widths = []
    print("Collecting 100 measurements (hold marker still)...")
    
    attempts = 0
    while len(widths) < 100 and attempts < 500:
        frame = grabber.read()
        if frame is None:
            time.sleep(0.01)
            attempts += 1
            continue
        
        detections = detector.detect(frame)
        target = None
        target_id = getattr(config, "ARUCO_TARGET_ID", 0)
        for d in detections:
            if d.get('id') == target_id:
                target = d
                break
        
        if target:
            pw = target.get('marker_width', target['bbox'][2])
            widths.append(pw)
            if len(widths) % 20 == 0:
                print(f"  Collected {len(widths)}/100 samples (latest width: {pw:.1f} px)")
        
        attempts += 1
        time.sleep(0.02)
    
    grabber.stop()
    
    if len(widths) < 10:
        print(f"\n❌ Only got {len(widths)} measurements. Make sure marker ID {target_id} is visible!")
        return
    
    mean_width = statistics.mean(widths)
    std_width = statistics.stdev(widths) if len(widths) > 1 else 0
    focal_length = (mean_width * distance_cm) / marker_size_cm
    
    results = {
        "marker_size_cm": marker_size_cm,
        "distance_cm": distance_cm,
        "mean_pixel_width": round(mean_width, 2),
        "std_pixel_width": round(std_width, 2),
        "num_samples": len(widths),
        "focal_length_px": round(focal_length, 2),
    }
    
    outfile = timestamp_filename("focal_length", "json")
    with open(outfile, "w") as f:
        json.dump(results, f, indent=2)
    
    print(f"\n{'─'*50}")
    print(f"  RESULTS:")
    print(f"  Mean pixel width:     {mean_width:.2f} ± {std_width:.2f} px")
    print(f"  Computed focal length: {focal_length:.2f} px")
    print(f"  Formula: f = ({mean_width:.1f} × {distance_cm}) / {marker_size_cm}")
    print(f"  Saved to: {outfile}")
    print(f"{'─'*50}")
    
    return results


# ═══════════════════════════════════════════════════════════════
# Item 3: Distance Estimation Accuracy
# ═══════════════════════════════════════════════════════════════
def test_distance_calibration():
    """
    Place marker at multiple known distances (20, 30, 40, 60, 100 cm).
    Measures pixel width at each, computes estimated distance using:
        d_est = (f × marker_size) / pixel_width
    Records accuracy at each range.
    """
    import cv2
    from aruco_detector import ArucoDetector
    from frame_grabber import FrameGrabber

    print("\n" + "="*60)
    print("  ITEM 3: DISTANCE ESTIMATION ACCURACY")
    print("="*60)
    
    focal_length = float(input("\nEnter your measured focal length (from Item 1, in px): "))
    marker_size_cm = float(input("Enter actual marker size in cm: "))
    
    distances = [20, 30, 40, 60, 100]
    print(f"\nYou will be prompted to place the marker at: {distances} cm")
    
    grabber = FrameGrabber()
    grabber.start()
    detector = ArucoDetector()
    
    results = []
    target_id = getattr(config, "ARUCO_TARGET_ID", 0)
    
    for dist_cm in distances:
        input(f"\n→ Place marker at exactly {dist_cm} cm, then press ENTER...")
        print(f"  Measuring at {dist_cm} cm (collecting 50 samples)...")
        
        widths = []
        attempts = 0
        while len(widths) < 50 and attempts < 300:
            frame = grabber.read()
            if frame is None:
                time.sleep(0.01)
                attempts += 1
                continue
            
            detections = detector.detect(frame)
            for d in detections:
                if d.get('id') == target_id:
                    widths.append(d.get('marker_width', d['bbox'][2]))
                    break
            attempts += 1
            time.sleep(0.02)
        
        if widths:
            mean_w = statistics.mean(widths)
            estimated_dist = (focal_length * marker_size_cm) / mean_w
            error_cm = estimated_dist - dist_cm
            error_pct = (error_cm / dist_cm) * 100
            
            results.append({
                "actual_cm": dist_cm,
                "mean_pixel_width": round(mean_w, 2),
                "estimated_cm": round(estimated_dist, 2),
                "error_cm": round(error_cm, 2),
                "error_pct": round(error_pct, 2),
                "num_samples": len(widths),
            })
            print(f"  ✓ Actual: {dist_cm} cm | Est: {estimated_dist:.1f} cm | Error: {error_cm:+.1f} cm ({error_pct:+.1f}%)")
        else:
            print(f"  ✗ No detections at {dist_cm} cm!")
            results.append({"actual_cm": dist_cm, "detected": False})
    
    grabber.stop()
    
    outfile = timestamp_filename("distance_calibration", "json")
    with open(outfile, "w") as f:
        json.dump(results, f, indent=2)
    
    print(f"\n{'─'*50}")
    print(f"  Results saved to: {outfile}")
    if results:
        valid = [r for r in results if "error_pct" in r]
        if valid:
            mae = statistics.mean([abs(r["error_cm"]) for r in valid])
            print(f"  Mean Absolute Error: {mae:.2f} cm")
    print(f"{'─'*50}")
    
    return results


# ═══════════════════════════════════════════════════════════════
# Item 4: Pipeline Latency Profiling
# ═══════════════════════════════════════════════════════════════
def test_latency():
    """
    Profiles each stage of the vision pipeline using time.perf_counter()
    over 1000 frames. Reports mean and std for:
      1. Frame grab
      2. ArUco detection
      3. Kalman filter step
      4. Navigator compute
      5. Command dispatch (to ESP32)
      6. Total frame time
    """
    import cv2
    from aruco_detector import ArucoDetector
    from frame_grabber import FrameGrabber
    from kalman_filter import ArucoKalmanFilter
    from navigator import Navigator
    from rover_comms import RoverComms

    print("\n" + "="*60)
    print("  ITEM 4: PIPELINE LATENCY PROFILING (1000 frames)")
    print("="*60)
    
    NUM_FRAMES = 1000
    
    grabber = FrameGrabber()
    grabber.start()
    detector = ArucoDetector()
    kalman = ArucoKalmanFilter()
    nav = Navigator()
    comms = RoverComms()
    
    # Warmup: skip first 30 frames
    print("Warming up camera (30 frames)...")
    for _ in range(30):
        frame = grabber.read()
        time.sleep(0.01)
    
    timings = {
        "frame_grab": [],
        "aruco_detect": [],
        "kalman_step": [],
        "nav_compute": [],
        "cmd_dispatch": [],
        "total": [],
    }
    
    target_id = getattr(config, "ARUCO_TARGET_ID", 0)
    print(f"Profiling {NUM_FRAMES} frames... (Point camera at ArUco marker ID {target_id})")
    
    for i in range(NUM_FRAMES):
        t_total_start = time.perf_counter()
        
        # Stage 1: Frame grab
        t0 = time.perf_counter()
        frame = grabber.read()
        t1 = time.perf_counter()
        timings["frame_grab"].append(t1 - t0)
        
        if frame is None:
            time.sleep(0.01)
            continue
        
        # Stage 2: ArUco detection
        t2 = time.perf_counter()
        detections = detector.detect(frame)
        t3 = time.perf_counter()
        timings["aruco_detect"].append(t3 - t2)
        
        # Find target
        target_det = None
        for d in detections:
            if d.get('id') == target_id:
                target_det = d
                break
        
        meas_bbox = target_det['bbox'] if target_det else None
        
        # Stage 3: Kalman filter step
        t4 = time.perf_counter()
        estimated_bbox, is_predicted = kalman.step(meas_bbox)
        t5 = time.perf_counter()
        timings["kalman_step"].append(t5 - t4)
        
        # Stage 4: Navigator compute
        t6 = time.perf_counter()
        if estimated_bbox:
            left_s, right_s = nav.compute_aruco_speeds(estimated_bbox, is_predicted=is_predicted)
        else:
            left_s, right_s = 0, 0
        t7 = time.perf_counter()
        timings["nav_compute"].append(t7 - t6)
        
        # Stage 5: Command dispatch (send 0,0 to avoid moving during benchmark)
        t8 = time.perf_counter()
        comms.send_command(0, 0)
        t9 = time.perf_counter()
        timings["cmd_dispatch"].append(t9 - t8)
        
        t_total_end = time.perf_counter()
        timings["total"].append(t_total_end - t_total_start)
        
        if (i + 1) % 200 == 0:
            avg_total_ms = statistics.mean(timings["total"][-200:]) * 1000
            print(f"  {i+1}/{NUM_FRAMES} frames | Avg total: {avg_total_ms:.2f} ms/frame")
    
    grabber.stop()
    comms.stop()
    
    # Compute statistics
    print(f"\n{'─'*65}")
    print(f"  {'Stage':<20} {'Mean (ms)':>10} {'Std (ms)':>10} {'Min (ms)':>10} {'Max (ms)':>10}")
    print(f"  {'─'*60}")
    
    results = {}
    for stage, times in timings.items():
        if not times:
            continue
        times_ms = [t * 1000 for t in times]
        mean_ms = statistics.mean(times_ms)
        std_ms = statistics.stdev(times_ms) if len(times_ms) > 1 else 0
        min_ms = min(times_ms)
        max_ms = max(times_ms)
        
        results[stage] = {
            "mean_ms": round(mean_ms, 3),
            "std_ms": round(std_ms, 3),
            "min_ms": round(min_ms, 3),
            "max_ms": round(max_ms, 3),
            "count": len(times_ms),
        }
        print(f"  {stage:<20} {mean_ms:>10.3f} {std_ms:>10.3f} {min_ms:>10.3f} {max_ms:>10.3f}")
    
    # Save raw timings
    outfile_csv = timestamp_filename("latency_raw", "csv")
    max_len = max(len(v) for v in timings.values())
    with open(outfile_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(list(timings.keys()))
        for i in range(max_len):
            row = []
            for stage in timings:
                if i < len(timings[stage]):
                    row.append(f"{timings[stage][i]*1000:.4f}")
                else:
                    row.append("")
            writer.writerow(row)
    
    outfile_json = timestamp_filename("latency_summary", "json")
    with open(outfile_json, "w") as f:
        json.dump(results, f, indent=2)
    
    print(f"\n  Raw data → {outfile_csv}")
    print(f"  Summary  → {outfile_json}")
    print(f"{'─'*65}")
    
    return results


# ═══════════════════════════════════════════════════════════════
# Item 5: Measured FPS
# ═══════════════════════════════════════════════════════════════
def test_fps():
    """
    Counts processed frames over 60 seconds in ArUco mode.
    Reports actual throughput FPS.
    """
    import cv2
    from aruco_detector import ArucoDetector
    from frame_grabber import FrameGrabber

    print("\n" + "="*60)
    print("  ITEM 5: MEASURED FPS (60s ArUco mode)")
    print("="*60)
    
    DURATION = 60.0
    
    grabber = FrameGrabber()
    grabber.start()
    detector = ArucoDetector()
    
    # Warmup
    print("Warming up camera...")
    for _ in range(20):
        grabber.read()
        time.sleep(0.01)
    
    print(f"Counting frames for {DURATION}s...")
    frame_count = 0
    detected_count = 0
    start = time.perf_counter()
    
    while True:
        elapsed = time.perf_counter() - start
        if elapsed >= DURATION:
            break
        
        frame = grabber.read()
        if frame is None:
            time.sleep(0.005)
            continue
        
        detections = detector.detect(frame)
        frame_count += 1
        if detections:
            detected_count += 1
        
        if frame_count % 200 == 0:
            current_fps = frame_count / elapsed
            print(f"  {elapsed:.1f}s | {frame_count} frames | {current_fps:.1f} FPS")
    
    actual_elapsed = time.perf_counter() - start
    grabber.stop()
    
    fps = frame_count / actual_elapsed
    detection_rate = (detected_count / frame_count * 100) if frame_count > 0 else 0
    
    results = {
        "duration_s": round(actual_elapsed, 2),
        "total_frames": frame_count,
        "fps": round(fps, 2),
        "frames_with_detection": detected_count,
        "detection_rate_pct": round(detection_rate, 1),
    }
    
    outfile = timestamp_filename("fps", "json")
    with open(outfile, "w") as f:
        json.dump(results, f, indent=2)
    
    print(f"\n{'─'*50}")
    print(f"  Total frames: {frame_count}")
    print(f"  Duration: {actual_elapsed:.2f} s")
    print(f"  MEASURED FPS: {fps:.2f}")
    print(f"  Detection rate: {detection_rate:.1f}%")
    print(f"  Saved to: {outfile}")
    print(f"{'─'*50}")
    
    return results


# ═══════════════════════════════════════════════════════════════
# Item 6: Kalman Filter Benefit (Raw vs Filtered)
# ═══════════════════════════════════════════════════════════════
def test_kalman_benefit():
    """
    Logs the raw ArUco detection position and the Kalman-filtered position
    side by side over 500 frames. Computes the standard deviation and RMSE
    improvement from filtering.
    """
    import cv2
    from aruco_detector import ArucoDetector
    from frame_grabber import FrameGrabber
    from kalman_filter import ArucoKalmanFilter

    print("\n" + "="*60)
    print("  ITEM 6: KALMAN FILTER BENEFIT (Raw vs Filtered)")
    print("="*60)
    
    NUM_FRAMES = 500
    
    grabber = FrameGrabber()
    grabber.start()
    detector = ArucoDetector()
    kalman = ArucoKalmanFilter()
    
    target_id = getattr(config, "ARUCO_TARGET_ID", 0)
    
    # Warmup
    for _ in range(20):
        grabber.read()
        time.sleep(0.01)
    
    print(f"Logging {NUM_FRAMES} frames... Keep marker visible and move it slowly.")
    
    raw_cx_list = []
    raw_cy_list = []
    filtered_cx_list = []
    filtered_cy_list = []
    timestamps = []
    
    count = 0
    start_time = time.perf_counter()
    
    while count < NUM_FRAMES:
        frame = grabber.read()
        if frame is None:
            time.sleep(0.01)
            continue
        
        detections = detector.detect(frame)
        target_det = None
        for d in detections:
            if d.get('id') == target_id:
                target_det = d
                break
        
        if target_det is None:
            continue  # Skip frames where marker not detected (we want apples-to-apples)
        
        raw_bbox = target_det['bbox']
        raw_cx = raw_bbox[0] + raw_bbox[2] / 2.0
        raw_cy = raw_bbox[1] + raw_bbox[3] / 2.0
        
        estimated_bbox, _ = kalman.step(raw_bbox)
        if estimated_bbox is None:
            continue
        
        filt_cx = estimated_bbox[0] + estimated_bbox[2] / 2.0
        filt_cy = estimated_bbox[1] + estimated_bbox[3] / 2.0
        
        t = time.perf_counter() - start_time
        
        raw_cx_list.append(raw_cx)
        raw_cy_list.append(raw_cy)
        filtered_cx_list.append(filt_cx)
        filtered_cy_list.append(filt_cy)
        timestamps.append(t)
        
        count += 1
        if count % 100 == 0:
            print(f"  {count}/{NUM_FRAMES} frames logged")
    
    grabber.stop()
    
    if len(raw_cx_list) < 20:
        print("❌ Not enough data collected!")
        return
    
    # Compute frame-to-frame jitter (first differences)
    raw_dx = np.diff(raw_cx_list)
    raw_dy = np.diff(raw_cy_list)
    filt_dx = np.diff(filtered_cx_list)
    filt_dy = np.diff(filtered_cy_list)
    
    raw_jitter = np.sqrt(raw_dx**2 + raw_dy**2)
    filt_jitter = np.sqrt(filt_dx**2 + filt_dy**2)
    
    raw_std_cx = statistics.stdev(raw_cx_list)
    raw_std_cy = statistics.stdev(raw_cy_list)
    filt_std_cx = statistics.stdev(filtered_cx_list)
    filt_std_cy = statistics.stdev(filtered_cy_list)
    
    results = {
        "num_frames": len(raw_cx_list),
        "raw_std_cx_px": round(raw_std_cx, 3),
        "raw_std_cy_px": round(raw_std_cy, 3),
        "filtered_std_cx_px": round(filt_std_cx, 3),
        "filtered_std_cy_px": round(filt_std_cy, 3),
        "raw_mean_jitter_px": round(float(np.mean(raw_jitter)), 3),
        "filtered_mean_jitter_px": round(float(np.mean(filt_jitter)), 3),
        "jitter_reduction_pct": round((1.0 - float(np.mean(filt_jitter)) / max(float(np.mean(raw_jitter)), 1e-9)) * 100, 1),
    }
    
    # Save raw CSV for plotting
    outfile_csv = timestamp_filename("kalman_raw_vs_filtered", "csv")
    with open(outfile_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_s", "raw_cx", "raw_cy", "filtered_cx", "filtered_cy"])
        for i in range(len(raw_cx_list)):
            writer.writerow([
                f"{timestamps[i]:.4f}",
                f"{raw_cx_list[i]:.2f}", f"{raw_cy_list[i]:.2f}",
                f"{filtered_cx_list[i]:.2f}", f"{filtered_cy_list[i]:.2f}",
            ])
    
    outfile_json = timestamp_filename("kalman_benefit", "json")
    with open(outfile_json, "w") as f:
        json.dump(results, f, indent=2)
    
    print(f"\n{'─'*60}")
    print(f"  {'Metric':<30} {'Raw':>12} {'Filtered':>12}")
    print(f"  {'─'*54}")
    print(f"  {'Std Dev X (px)':<30} {raw_std_cx:>12.3f} {filt_std_cx:>12.3f}")
    print(f"  {'Std Dev Y (px)':<30} {raw_std_cy:>12.3f} {filt_std_cy:>12.3f}")
    print(f"  {'Mean Frame Jitter (px)':<30} {np.mean(raw_jitter):>12.3f} {np.mean(filt_jitter):>12.3f}")
    print(f"  {'Jitter Reduction':<30} {'':>12} {results['jitter_reduction_pct']:>11.1f}%")
    print(f"\n  Raw data → {outfile_csv}")
    print(f"  Summary  → {outfile_json}")
    print(f"{'─'*60}")
    
    return results


# ═══════════════════════════════════════════════════════════════
# Item 7: Step Response (MOST IMPORTANT FOR CONTROL SYSTEMS)
# ═══════════════════════════════════════════════════════════════
def test_step_response():
    """
    Step Response Test — the most important result for a Control Systems course.
    
    Procedure:
    1. Place marker in the camera's dead zone (centered at target distance).
       Record the steady state.
    2. Suddenly move the marker to one side (step input in angular).
       OR suddenly move it closer/farther (step input in linear distance).
    3. The script logs the error signal and motor output at each frame,
       then computes: rise time, overshoot, settling time, steady-state error.
    
    Output: CSV of (time, angular_error, linear_error, left_pwm, right_pwm, marker_cx, marker_w)
    """
    import cv2
    from aruco_detector import ArucoDetector
    from frame_grabber import FrameGrabber
    from kalman_filter import ArucoKalmanFilter
    from navigator import Navigator
    from rover_comms import RoverComms

    print("\n" + "="*60)
    print("  ITEM 7: STEP RESPONSE (Control Systems)")
    print("="*60)
    print("""
    This is the most important measurement for your review.
    
    PROCEDURE:
    1. Place the marker centered at target distance (~35-40cm).
       Press ENTER to start recording the baseline (steady state).
    2. After 3 seconds, QUICKLY move the marker to one side
       (or change distance). The script records the response.
    3. Keep recording for 10+ seconds until the rover settles.
    4. Press Ctrl+C to stop and compute metrics.
    
    The script controls the motors in REAL TIME during this test!
    """)
    
    input("Press ENTER when marker is centered and ready...")
    
    grabber = FrameGrabber()
    grabber.start()
    detector = ArucoDetector()
    kalman = ArucoKalmanFilter()
    nav = Navigator()
    comms = RoverComms()
    
    target_id = getattr(config, "ARUCO_TARGET_ID", 0)
    frame_w = float(config.FRAME_WIDTH)
    frame_center = frame_w / 2.0
    target_width = float(getattr(config, "ARUCO_TARGET_WIDTH", 140))
    
    log_data = []
    
    print("\n🔴 RECORDING... Move the marker after ~3s. Press Ctrl+C to stop.\n")
    
    start_time = time.perf_counter()
    
    try:
        while True:
            frame = grabber.read()
            if frame is None:
                time.sleep(0.005)
                continue
            
            t = time.perf_counter() - start_time
            
            detections = detector.detect(frame)
            target_det = None
            for d in detections:
                if d.get('id') == target_id:
                    target_det = d
                    break
            
            meas_bbox = target_det['bbox'] if target_det else None
            estimated_bbox, is_predicted = kalman.step(meas_bbox)
            
            if estimated_bbox:
                left_s, right_s = nav.compute_aruco_speeds(estimated_bbox, is_predicted=is_predicted)
                comms.send_command(left_s, right_s)
                
                cx = estimated_bbox[0] + estimated_bbox[2] / 2.0
                w = estimated_bbox[2]
                
                # Compute errors the same way navigator does
                horizontal_offset = cx - frame_center
                deadzone_px = float(getattr(config, "ARUCO_DEADZONE_X", 35))
                if abs(horizontal_offset) < deadzone_px:
                    angular_error = 0.0
                else:
                    sign = 1.0 if horizontal_offset > 0 else -1.0
                    angular_error = (horizontal_offset - sign * deadzone_px) / (frame_center - deadzone_px)
                    angular_error = max(min(angular_error, 1.0), -1.0)
                
                width_margin = float(getattr(config, "ARUCO_WIDTH_MARGIN", 15))
                width_error = target_width - w
                if abs(width_error) < width_margin:
                    linear_error = 0.0
                else:
                    sign = 1.0 if width_error > 0 else -1.0
                    linear_error = (width_error - sign * width_margin) / target_width
                    linear_error = max(min(linear_error, 1.0), -1.0)
                
                log_data.append({
                    "t": round(t, 4),
                    "angular_error": round(angular_error, 4),
                    "linear_error": round(linear_error, 4),
                    "left_pwm": left_s,
                    "right_pwm": right_s,
                    "marker_cx": round(cx, 1),
                    "marker_w": round(w, 1),
                    "is_predicted": is_predicted,
                })
                
                if len(log_data) % 50 == 0:
                    print(f"  t={t:.1f}s | AngErr={angular_error:+.3f} | LinErr={linear_error:+.3f} | L={left_s} R={right_s} | cx={cx:.0f} w={w:.0f}")
            else:
                comms.send_command(0, 0)
            
            time.sleep(0.02)
    
    except KeyboardInterrupt:
        print("\n\n⏹ Recording stopped.")
    
    comms.send_command(0, 0)
    comms.stop()
    grabber.stop()
    
    if len(log_data) < 10:
        print("❌ Not enough data! Need at least 10 frames.")
        return
    
    # Save raw CSV
    outfile_csv = timestamp_filename("step_response", "csv")
    with open(outfile_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=log_data[0].keys())
        writer.writeheader()
        writer.writerows(log_data)
    
    # Compute step response metrics (angular channel)
    errors = [d["angular_error"] for d in log_data]
    times = [d["t"] for d in log_data]
    
    # Find the step: largest jump in error between consecutive samples
    diffs = [abs(errors[i+1] - errors[i]) for i in range(len(errors)-1)]
    step_idx = diffs.index(max(diffs))
    
    if step_idx > 0:
        initial_error = statistics.mean(errors[:step_idx])  # pre-step baseline
    else:
        initial_error = errors[0]
    
    peak_error = max(abs(e) for e in errors[step_idx:])
    
    # Steady-state error (last 20% of recording)
    tail_start = int(len(errors) * 0.8)
    ss_error = statistics.mean([abs(e) for e in errors[tail_start:]])
    
    # Rise time: time to go from 10% to 90% of the step
    step_magnitude = peak_error - abs(initial_error)
    if step_magnitude > 0.01:
        threshold_10 = abs(initial_error) + 0.1 * step_magnitude
        threshold_90 = abs(initial_error) + 0.9 * step_magnitude
        
        t_10 = None
        t_90 = None
        for i in range(step_idx, len(errors)):
            err_abs = abs(errors[i])
            if t_10 is None and err_abs >= threshold_10:
                t_10 = times[i]
            if t_90 is None and err_abs >= threshold_90:
                t_90 = times[i]
        
        rise_time = (t_90 - t_10) if (t_10 is not None and t_90 is not None) else None
    else:
        rise_time = None
    
    # Settling time: last time error exceeds 5% of peak
    settling_band = 0.05 * peak_error if peak_error > 0 else 0.02
    settling_time = None
    for i in range(len(errors)-1, step_idx, -1):
        if abs(errors[i]) > settling_band:
            settling_time = times[i] - times[step_idx]
            break
    
    # Overshoot: how much the error exceeds the step magnitude (for undershoot recovery)
    overshoot_pct = None
    if step_magnitude > 0.01:
        # After error peaks, does it go past zero (overshoot)?
        post_peak = errors[step_idx:]
        signed_peak = max(post_peak, key=abs)
        # Check for sign change (overshoot)
        crossed_zero = False
        for i in range(1, len(post_peak)):
            if (post_peak[i-1] > 0 and post_peak[i] < 0) or (post_peak[i-1] < 0 and post_peak[i] > 0):
                crossed_zero = True
                opposite_peak = max(abs(e) for e in post_peak[i:]) if i < len(post_peak) else 0
                overshoot_pct = (opposite_peak / abs(signed_peak)) * 100 if abs(signed_peak) > 0 else 0
                break
        if not crossed_zero:
            overshoot_pct = 0.0
    
    metrics = {
        "num_samples": len(log_data),
        "duration_s": round(times[-1] - times[0], 2),
        "step_detected_at_s": round(times[step_idx], 3),
        "peak_angular_error": round(peak_error, 4),
        "steady_state_error": round(ss_error, 4),
        "rise_time_s": round(rise_time, 4) if rise_time else "N/A",
        "settling_time_s": round(settling_time, 4) if settling_time else "N/A",
        "overshoot_pct": round(overshoot_pct, 2) if overshoot_pct is not None else "N/A",
    }
    
    outfile_json = timestamp_filename("step_response_metrics", "json")
    with open(outfile_json, "w") as f:
        json.dump(metrics, f, indent=2)
    
    print(f"\n{'─'*55}")
    print(f"  STEP RESPONSE METRICS (Angular Channel)")
    print(f"  {'─'*50}")
    print(f"  Step detected at:      {metrics['step_detected_at_s']} s")
    print(f"  Peak error:            {metrics['peak_angular_error']}")
    print(f"  Rise time (10→90%):    {metrics['rise_time_s']} s")
    print(f"  Settling time (5%):    {metrics['settling_time_s']} s")
    print(f"  Overshoot:             {metrics['overshoot_pct']}%")
    print(f"  Steady-state error:    {metrics['steady_state_error']}")
    print(f"\n  Raw data  → {outfile_csv}")
    print(f"  Metrics   → {outfile_json}")
    print(f"{'─'*55}")
    
    return metrics


# ═══════════════════════════════════════════════════════════════
# Item 8: Breakaway PWM
# ═══════════════════════════════════════════════════════════════
def test_breakaway_pwm():
    """
    Slowly ramps PWM from 0 to 150, pausing at each step.
    User presses ENTER when they see wheels start turning.
    Records the breakaway PWM for each wheel.
    """
    from rover_comms import RoverComms

    print("\n" + "="*60)
    print("  ITEM 8: BREAKAWAY PWM SWEEP")
    print("="*60)
    print("""
    This test ramps PWM from 0 to 150 in steps of 5.
    Watch the rover wheels closely on the FLOOR (under load).
    Press ENTER as soon as BOTH wheels start turning.
    """)
    
    comms = RoverComms()
    time.sleep(1)  # Let connection establish
    
    results = {}
    
    for direction, label in [(1, "FORWARD"), (-1, "REVERSE")]:
        print(f"\n{'─'*40}")
        print(f"  Testing {label} direction")
        print(f"{'─'*40}")
        
        input(f"  Place rover on floor. Press ENTER to start {label} sweep...")
        
        breakaway_pwm = None
        for pwm in range(0, 155, 5):
            actual_pwm = pwm * direction
            comms.send_command(actual_pwm, actual_pwm)
            print(f"    PWM = {pwm:>3}  (sending {actual_pwm:>4}, {actual_pwm:>4})")
            
            # Wait and check user input
            time.sleep(0.8)
            
            # Non-blocking check: just ask if wheels are moving
            try:
                response = input("    Wheels moving? (y/ENTER to continue): ").strip().lower()
                if response == 'y':
                    breakaway_pwm = pwm
                    print(f"    ✓ Breakaway at PWM = {pwm}")
                    break
            except EOFError:
                pass
        
        comms.send_command(0, 0)
        results[label.lower()] = breakaway_pwm or "not reached"
    
    comms.stop()
    
    outfile = timestamp_filename("breakaway_pwm", "json")
    with open(outfile, "w") as f:
        json.dump(results, f, indent=2)
    
    print(f"\n{'─'*50}")
    print(f"  BREAKAWAY PWM RESULTS:")
    print(f"  Forward: {results.get('forward', 'N/A')}")
    print(f"  Reverse: {results.get('reverse', 'N/A')}")
    print(f"  Saved to: {outfile}")
    print(f"{'─'*50}")
    
    return results


# ═══════════════════════════════════════════════════════════════
# Item 9: Wheel Speed at Specific PWMs
# ═══════════════════════════════════════════════════════════════
def test_wheel_speed():
    """
    Drives the rover at specific PWMs (85, 120, 170) for a measured distance.
    User times it manually with a stopwatch and enters the time.
    Computes velocity in cm/s.
    """
    from rover_comms import RoverComms

    print("\n" + "="*60)
    print("  ITEM 9: WHEEL SPEED MEASUREMENT")
    print("="*60)
    print("""
    Mark a straight-line distance on the floor (e.g., 100 cm).
    Place the rover at the start.
    The script will drive at each PWM — you time it.
    """)
    
    distance_cm = float(input("\nEnter the marked distance in cm (e.g., 100): "))
    test_pwms = [85, 120, 170]
    
    comms = RoverComms()
    time.sleep(1)
    
    results = []
    
    for pwm in test_pwms:
        print(f"\n{'─'*40}")
        print(f"  Testing PWM = {pwm}")
        print(f"{'─'*40}")
        
        input(f"  Place rover at START of {distance_cm}cm track. Press ENTER to GO...")
        
        start_t = time.perf_counter()
        comms.send_command(pwm, pwm)
        
        input("  Press ENTER when rover reaches the END of the track...")
        end_t = time.perf_counter()
        comms.send_command(0, 0)
        
        elapsed = end_t - start_t
        speed_cm_s = distance_cm / elapsed
        
        # If you know wheel diameter, compute RPM
        wheel_diam_cm = float(input("  Enter wheel diameter in cm (or 0 to skip RPM calc): ") or "0")
        rpm = 0
        if wheel_diam_cm > 0:
            circumference_cm = np.pi * wheel_diam_cm
            rps = speed_cm_s / circumference_cm
            rpm = rps * 60
        
        result = {
            "pwm": pwm,
            "distance_cm": distance_cm,
            "time_s": round(elapsed, 3),
            "speed_cm_s": round(speed_cm_s, 2),
            "rpm": round(rpm, 1) if rpm > 0 else "N/A",
        }
        results.append(result)
        
        print(f"  ✓ Time: {elapsed:.3f}s | Speed: {speed_cm_s:.2f} cm/s" + 
              (f" | RPM: {rpm:.1f}" if rpm > 0 else ""))
    
    comms.stop()
    
    outfile = timestamp_filename("wheel_speed", "json")
    with open(outfile, "w") as f:
        json.dump(results, f, indent=2)
    
    print(f"\n{'─'*60}")
    print(f"  {'PWM':<8} {'Time (s)':<12} {'Speed (cm/s)':<15} {'RPM':<10}")
    print(f"  {'─'*50}")
    for r in results:
        print(f"  {r['pwm']:<8} {r['time_s']:<12} {r['speed_cm_s']:<15} {r['rpm']}")
    print(f"\n  Saved to: {outfile}")
    print(f"{'─'*60}")
    
    return results


# ═══════════════════════════════════════════════════════════════
# Item 10: Maximum Yaw Rate
# ═══════════════════════════════════════════════════════════════
def test_yaw_rate():
    """
    Spins the rover at maximum angular command and times a full 360° turn.
    """
    from rover_comms import RoverComms

    print("\n" + "="*60)
    print("  ITEM 10: MAXIMUM YAW RATE")
    print("="*60)
    print("""
    Mark a reference point on the floor aligned with the rover's front.
    The script will spin the rover at max speed.
    Press ENTER when it completes exactly ONE full 360° rotation.
    """)
    
    comms = RoverComms()
    time.sleep(1)
    
    pwm = config.MAX_SPEED
    
    results = []
    for direction, label in [(1, "Clockwise (Left+, Right-)"), (-1, "Counter-clockwise")]:
        input(f"\n  Align rover to reference mark. Press ENTER to start {label}...")
        
        start_t = time.perf_counter()
        comms.send_command(pwm * direction, -pwm * direction)
        
        input("  Press ENTER when the rover completes 360°...")
        end_t = time.perf_counter()
        comms.send_command(0, 0)
        
        elapsed = end_t - start_t
        yaw_rate_dps = 360.0 / elapsed  # degrees per second
        yaw_rate_rps = yaw_rate_dps / 360.0  # revolutions per second
        
        results.append({
            "direction": label,
            "pwm": pwm,
            "time_for_360_s": round(elapsed, 3),
            "yaw_rate_deg_per_s": round(yaw_rate_dps, 2),
            "yaw_rate_rev_per_s": round(yaw_rate_rps, 3),
        })
        
        print(f"  ✓ 360° in {elapsed:.3f}s | {yaw_rate_dps:.1f}°/s | {yaw_rate_rps:.3f} rev/s")
    
    comms.stop()
    
    outfile = timestamp_filename("yaw_rate", "json")
    with open(outfile, "w") as f:
        json.dump(results, f, indent=2)
    
    print(f"\n  Saved to: {outfile}")
    return results


# ═══════════════════════════════════════════════════════════════
# Item 13: Occlusion Recovery Test
# ═══════════════════════════════════════════════════════════════
def test_occlusion():
    """
    Tests Kalman filter occlusion recovery at different durations.
    Hides the marker for 0.5, 1.0, 1.5, 2.0 seconds and checks
    whether tracking recovers and how fast.
    """
    import cv2
    from aruco_detector import ArucoDetector
    from frame_grabber import FrameGrabber
    from kalman_filter import ArucoKalmanFilter

    print("\n" + "="*60)
    print("  ITEM 13: OCCLUSION RECOVERY TEST")
    print("="*60)
    print("""
    Keep the marker visible. The script will track it, then ask
    you to HIDE it for specific durations.
    
    After each hide duration, reveal the marker again.
    The script checks if tracking recovers.
    """)
    
    grabber = FrameGrabber()
    grabber.start()
    detector = ArucoDetector()
    
    target_id = getattr(config, "ARUCO_TARGET_ID", 0)
    test_durations = [0.5, 1.0, 1.5, 2.0]
    results = []
    
    for hide_dur in test_durations:
        print(f"\n{'─'*40}")
        print(f"  Test: {hide_dur}s occlusion")
        print(f"{'─'*40}")
        
        kalman = ArucoKalmanFilter()  # Fresh filter per test
        
        # Phase 1: Establish tracking (need 30 good frames)
        input(f"  Show marker ID {target_id} to camera. Press ENTER when ready...")
        print("  Establishing tracking (30 frames)...")
        
        good_frames = 0
        while good_frames < 30:
            frame = grabber.read()
            if frame is None:
                time.sleep(0.01)
                continue
            detections = detector.detect(frame)
            for d in detections:
                if d.get('id') == target_id:
                    kalman.step(d['bbox'])
                    good_frames += 1
                    break
            time.sleep(0.02)
        
        last_seen_bbox = kalman.get_bbox()
        print(f"  ✓ Tracking established. Last bbox: {last_seen_bbox}")
        
        # Phase 2: Occlude for specified duration
        print(f"\n  ⚠️  HIDE THE MARKER NOW! Keep it hidden for {hide_dur}s...")
        input("  Press ENTER the moment you hide it...")
        
        hide_start = time.perf_counter()
        predictions_during_occlusion = []
        
        while time.perf_counter() - hide_start < hide_dur:
            frame = grabber.read()
            if frame is None:
                time.sleep(0.01)
                continue
            
            # During occlusion, we expect NO detection
            detections = detector.detect(frame)
            target_in_frame = any(d.get('id') == target_id for d in detections)
            
            estimated, is_pred = kalman.step(None)  # Force no measurement
            if estimated:
                predictions_during_occlusion.append({
                    "t": round(time.perf_counter() - hide_start, 3),
                    "bbox": estimated,
                    "is_predicted": is_pred,
                    "actually_visible": target_in_frame,
                })
            time.sleep(0.02)
        
        actual_hide_time = time.perf_counter() - hide_start
        
        # Phase 3: Reveal and measure recovery
        print(f"  ✓ Hidden for {actual_hide_time:.2f}s. REVEAL THE MARKER NOW!")
        
        recovery_start = time.perf_counter()
        recovered = False
        recovery_time = None
        max_recovery_wait = 5.0
        
        while time.perf_counter() - recovery_start < max_recovery_wait:
            frame = grabber.read()
            if frame is None:
                time.sleep(0.01)
                continue
            
            detections = detector.detect(frame)
            for d in detections:
                if d.get('id') == target_id:
                    estimated, is_pred = kalman.step(d['bbox'])
                    if estimated and not is_pred:
                        recovered = True
                        recovery_time = time.perf_counter() - recovery_start
                        break
            
            if recovered:
                break
            
            # Also step kalman with no detection to see if filter survived
            kalman.step(None)
            time.sleep(0.02)
        
        result = {
            "hide_duration_target_s": hide_dur,
            "hide_duration_actual_s": round(actual_hide_time, 3),
            "predictions_during_occlusion": len(predictions_during_occlusion),
            "filter_survived": kalman.is_initialized,
            "recovered": recovered,
            "recovery_time_s": round(recovery_time, 3) if recovery_time else "N/A",
        }
        results.append(result)
        
        status = "✓ RECOVERED" if recovered else "✗ LOST"
        print(f"  {status} | Recovery time: {result['recovery_time_s']}s | Filter alive: {kalman.is_initialized}")
    
    grabber.stop()
    
    outfile = timestamp_filename("occlusion_recovery", "json")
    with open(outfile, "w") as f:
        json.dump(results, f, indent=2)
    
    print(f"\n{'─'*65}")
    print(f"  {'Hide (s)':<12} {'Actual (s)':<12} {'Recovered':<12} {'Recovery (s)':<15} {'Filter'}")
    print(f"  {'─'*60}")
    for r in results:
        print(f"  {r['hide_duration_target_s']:<12} {r['hide_duration_actual_s']:<12} "
              f"{'Yes' if r['recovered'] else 'No':<12} {r['recovery_time_s']:<15} "
              f"{'Alive' if r['filter_survived'] else 'Reset'}")
    print(f"\n  Saved to: {outfile}")
    print(f"{'─'*65}")
    
    return results


# ═══════════════════════════════════════════════════════════════
# Item 15: Watchdog Stop Time
# ═══════════════════════════════════════════════════════════════
def test_watchdog():
    """
    Drives the rover at a steady speed, then abruptly stops
    sending commands. Measures how long the motors take to stop
    (should be ~WATCHDOG_TIMEOUT_MS from config).
    """
    from rover_comms import RoverComms

    print("\n" + "="*60)
    print("  ITEM 15: WATCHDOG STOP TIME")
    print("="*60)
    print("""
    The rover will drive forward for 3 seconds, then the script
    will STOP sending commands and rely on the ESP32 watchdog
    to cut the motors.
    
    Watch the rover and press ENTER when it actually stops.
    Expected: ~500ms (from WATCHDOG_TIMEOUT_MS).
    """)
    
    comms = RoverComms()
    time.sleep(1)
    
    pwm = 120
    
    input("Place rover on a clear path. Press ENTER to start...")
    
    # Drive for 3 seconds
    print(f"  Driving at PWM={pwm} for 3 seconds...")
    drive_start = time.perf_counter()
    while time.perf_counter() - drive_start < 3.0:
        comms.send_command(pwm, pwm)
        time.sleep(0.05)
    
    # STOP sending commands (don't send 0,0 — let watchdog handle it)
    print(f"  ⚡ COMMANDS STOPPED at t={time.perf_counter() - drive_start:.3f}s")
    print(f"  Watchdog should stop motors in ~{config.WATCHDOG_TIMEOUT_MS}ms")
    stop_cmd_time = time.perf_counter()
    
    input("  Press ENTER the moment the rover fully stops...")
    actual_stop_time = time.perf_counter()
    
    measured_stop_ms = (actual_stop_time - stop_cmd_time) * 1000
    
    results = {
        "pwm_before_stop": pwm,
        "watchdog_timeout_config_ms": config.WATCHDOG_TIMEOUT_MS,
        "measured_stop_time_ms": round(measured_stop_ms, 1),
        "note": "Human reaction time adds ~200-300ms to this measurement",
    }
    
    comms.send_command(0, 0)
    comms.stop()
    
    outfile = timestamp_filename("watchdog_stop", "json")
    with open(outfile, "w") as f:
        json.dump(results, f, indent=2)
    
    print(f"\n{'─'*50}")
    print(f"  Config watchdog timeout:   {config.WATCHDOG_TIMEOUT_MS} ms")
    print(f"  Measured stop time:        {measured_stop_ms:.1f} ms")
    print(f"  (includes human reaction time ~200-300ms)")
    print(f"  Saved to: {outfile}")
    print(f"{'─'*50}")
    
    return results


# ═══════════════════════════════════════════════════════════════
# Item 17: Wi-Fi Command Round-Trip Time
# ═══════════════════════════════════════════════════════════════
def test_wifi_rtt():
    """
    Sends 200 HTTP requests to the ESP32 /control and /status endpoints.
    Measures the round-trip time for each.
    """
    import requests

    print("\n" + "="*60)
    print("  ITEM 17: WI-FI COMMAND ROUND-TRIP TIME")
    print("="*60)
    
    base_url = config.ESP8266_MOTOR_URL.rstrip("/")
    if not base_url.startswith("http"):
        base_url = f"http://{base_url}"
    
    NUM_REQUESTS = 200
    
    session = requests.Session()
    adapter = requests.adapters.HTTPAdapter(pool_connections=1, pool_maxsize=2)
    session.mount("http://", adapter)
    session.trust_env = False
    
    # Warmup
    print(f"Testing against {base_url}...")
    try:
        session.get(f"{base_url}/status", timeout=2)
    except Exception as e:
        print(f"❌ Cannot reach ESP32: {e}")
        print("Make sure ESP32 is powered on and connected to WiFi.")
        return
    
    # Test /control endpoint
    print(f"\nMeasuring {NUM_REQUESTS} round-trips to /control...")
    control_rtts = []
    for i in range(NUM_REQUESTS):
        t0 = time.perf_counter()
        try:
            resp = session.get(f"{base_url}/control?left=0&right=0", timeout=2)
            t1 = time.perf_counter()
            if resp.status_code == 200:
                control_rtts.append((t1 - t0) * 1000)
        except Exception:
            pass
        
        if (i + 1) % 50 == 0:
            avg = statistics.mean(control_rtts[-50:]) if control_rtts else 0
            print(f"  {i+1}/{NUM_REQUESTS} | Avg RTT: {avg:.2f} ms")
    
    # Test /status endpoint
    print(f"\nMeasuring {NUM_REQUESTS} round-trips to /status...")
    status_rtts = []
    for i in range(NUM_REQUESTS):
        t0 = time.perf_counter()
        try:
            resp = session.get(f"{base_url}/status", timeout=2)
            t1 = time.perf_counter()
            if resp.status_code == 200:
                status_rtts.append((t1 - t0) * 1000)
        except Exception:
            pass
        
        if (i + 1) % 50 == 0:
            avg = statistics.mean(status_rtts[-50:]) if status_rtts else 0
            print(f"  {i+1}/{NUM_REQUESTS} | Avg RTT: {avg:.2f} ms")
    
    session.close()
    
    results = {}
    for endpoint, rtts in [("control", control_rtts), ("status", status_rtts)]:
        if rtts:
            results[endpoint] = {
                "num_requests": len(rtts),
                "mean_ms": round(statistics.mean(rtts), 3),
                "std_ms": round(statistics.stdev(rtts), 3) if len(rtts) > 1 else 0,
                "min_ms": round(min(rtts), 3),
                "max_ms": round(max(rtts), 3),
                "median_ms": round(statistics.median(rtts), 3),
                "p95_ms": round(sorted(rtts)[int(len(rtts) * 0.95)], 3),
                "p99_ms": round(sorted(rtts)[int(len(rtts) * 0.99)], 3),
            }
    
    outfile = timestamp_filename("wifi_rtt", "json")
    with open(outfile, "w") as f:
        json.dump(results, f, indent=2)
    
    print(f"\n{'─'*65}")
    print(f"  {'Endpoint':<12} {'Mean':>8} {'Std':>8} {'Min':>8} {'Median':>8} {'P95':>8} {'Max':>8}")
    print(f"  {'─'*60}")
    for ep, data in results.items():
        print(f"  /{ep:<11} {data['mean_ms']:>7.2f} {data['std_ms']:>7.2f} "
              f"{data['min_ms']:>7.2f} {data['median_ms']:>7.2f} {data['p95_ms']:>7.2f} {data['max_ms']:>7.2f}")
    print(f"\n  Saved to: {outfile}")
    print(f"{'─'*65}")
    
    return results


# ═══════════════════════════════════════════════════════════════
# Main CLI
# ═══════════════════════════════════════════════════════════════
TEST_MAP = {
    "focal_length": test_focal_length,
    "distance_cal": test_distance_calibration,
    "latency": test_latency,
    "fps": test_fps,
    "kalman_benefit": test_kalman_benefit,
    "step_response": test_step_response,
    "breakaway_pwm": test_breakaway_pwm,
    "wheel_speed": test_wheel_speed,
    "yaw_rate": test_yaw_rate,
    "occlusion": test_occlusion,
    "watchdog": test_watchdog,
    "wifi_rtt": test_wifi_rtt,
}

def main():
    parser = argparse.ArgumentParser(
        description="Rover Benchmark & Instrumentation Suite",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        "--test", "-t",
        choices=list(TEST_MAP.keys()) + ["all", "priority"],
        required=True,
        help="Which test to run"
    )
    
    args = parser.parse_args()
    
    if args.test == "all":
        # Non-motor tests that can run sequentially
        print("\n" + "="*60)
        print("  RUNNING ALL NON-MOTOR TESTS")
        print("="*60)
        for test_name in ["focal_length", "latency", "fps", "wifi_rtt"]:
            try:
                TEST_MAP[test_name]()
            except Exception as e:
                print(f"\n❌ Test '{test_name}' failed: {e}")
    
    elif args.test == "priority":
        # Priority tests (Items 1, 4, 5, 8, 9, 13)
        print("\n" + "="*60)
        print("  RUNNING PRIORITY TESTS (Items 1, 4, 5, 8, 9, 13)")
        print("="*60)
        for test_name in ["focal_length", "latency", "fps", "breakaway_pwm", "wheel_speed", "occlusion"]:
            try:
                TEST_MAP[test_name]()
            except KeyboardInterrupt:
                print(f"\n⏹ Test '{test_name}' interrupted.")
            except Exception as e:
                print(f"\n❌ Test '{test_name}' failed: {e}")
    
    else:
        TEST_MAP[args.test]()
    
    print("\n✅ All selected tests complete!")
    print(f"   Results saved in: {os.path.abspath(RESULTS_DIR)}")


if __name__ == "__main__":
    main()
