# Rover Control Android App

Companion remote control and telemetry app built with **Jetpack Compose**, **OkHttp WebSockets**, and **CameraX**.

---

## 📱 Features
- **Live WebSocket Telemetry**: Connects to the companion PC (`ws://<laptop-ip>:8765`) to display tracking state (`IDLE`, `TRACKING`, `SEARCHING`, `LOST`).
- **Touch Virtual Joystick**: Smooth bi-directional manual steering when autonomous following is paused.
- **Reference Target Registration**: Uses CameraX to snap multiple photos of a target object and streams base64 images directly to the PC for multi-angle visual matching.
- **Start / Stop Controls**: Single-tap auto-follow engagement and emergency motor cutoff.

---

## 🛠️ How to Open & Run in Android Studio

### 1. Prerequisites
- **Android Studio** (Hedgehog 2023.1.1 or newer recommended).
- Android device or emulator running **Android 7.0 (API 24)** or higher.
- Both phone and laptop connected to the same Wi-Fi network / hotspot.

### 2. Opening the Project
1. Open Android Studio.
2. Select **File > Open...** (or click **Open** on the Welcome screen).
3. Navigate to and select the `android_app` folder:
   ```
   Vision-Based-Object-Following-Rover/android_app
   ```
4. Click **OK**.

### 3. Gradle Sync
1. Wait for Android Studio to automatically index and sync Gradle dependencies.
2. If prompted, click **Sync Project with Gradle Files** (the elephant icon in the top right toolbar).
3. Ensure JDK version is set to Java 17 (or Java 11) in **Settings > Build, Execution, Deployment > Build Tools > Gradle > Gradle JDK**.

### 4. Running the App on Your Phone
1. Enable **Developer Options** and **USB Debugging** on your Android phone.
2. Connect your phone via USB (or pair over Wi-Fi debugging).
3. Select your phone in the top device dropdown list.
4. Click the green **Run (▶)** button (or press `Shift + F10`).
5. Android Studio will compile, install, and launch the app directly on your phone.

---

## 🌐 Connecting to the Rover Server
1. Launch the server dashboard on your laptop:
   ```bash
   cd server
   python main.py
   ```
2. Note your laptop's local IP address (e.g., `192.168.29.204`).
3. Open the **Rover Control** app on your phone, type the IP into the connection bar, and tap **Connect**.
4. The dashboard header will turn green indicating `📱 Mobile Connected`.
